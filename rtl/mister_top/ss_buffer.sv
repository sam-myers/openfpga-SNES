// Save state buffer
//
// The upstream savestate engine expects a 64-bit DDR3 port (MiSTer). The Pocket has no DDR3, so this
// module services those requests out of SDRAM, using the controller's SNI channel 1 (banks 2/3, which
// never hold ROM data). SNI waits for gaps between CPU ROM accesses, so it is safe to use while the
// SNES is running.
//
// A second 16-bit host port lets the APF bridge read the finished snapshot out of, or write a
// snapshot into, the same buffer. The two ports are never active at the same time in practice, but
// the engine port takes priority.
//
// Buffer layout matches MiSTer's DDR layout for slot 0: byte N of the save state file lives at
// SDRAM channel 1 byte address N.
module ss_buffer (
    input wire clk_sys,
    input wire clk_mem,

    // Savestate engine (clk_sys)
    input wire ddr_req,
    input wire ddr_we,
    input wire [21:3] ddr_addr,
    input wire [63:0] ddr_do,
    input wire [7:0] ddr_be,
    output reg ddr_ack = 0,
    // Written in clk_mem, but stable before ddr_ack toggles
    output reg [63:0] ddr_di = 0,

    // Host port (clk_mem). Pulse host_req for one cycle and wait for host_ack before the next request
    input wire host_req,
    input wire host_we,
    input wire [19:1] host_addr,
    input wire [15:0] host_din,
    output reg [15:0] host_dout = 0,
    output reg host_ack = 0,

    // SDRAM SNI (clk_mem)
    output reg [24:0] sni_addr = 0,
    output reg [15:0] sni_din = 0,
    output reg sni_word = 1,
    output reg sni_rd_req = 0,
    output reg sni_wr_req = 0,
    input wire [15:0] sni_dout,
    input wire sni_ready,

    // Debug counters (clk_mem). Only meaningful once the buffer is idle
    output reg [31:0] dbg_eng_ops = 0,
    output reg [31:0] dbg_eng_writes = 0,
    output reg [63:0] dbg_hdr0 = 0,
    output reg [63:0] dbg_hdr8 = 0
);
  ////////////////////////////////////////////////////////////////////////////////////////
  // Engine request capture (clk_sys)

  // ddr_we and ddr_be are only valid on the cycle ddr_req toggles, so latch everything then
  reg req_seen = 0;
  reg cap_toggle = 0;
  reg cap_we = 0;
  reg [21:3] cap_addr = 0;
  reg [63:0] cap_do = 0;
  reg [7:0] cap_be = 0;

  always @(posedge clk_sys) begin
    if (ddr_req != req_seen) begin
      req_seen <= ddr_req;

      cap_we <= ddr_we;
      cap_addr <= ddr_addr;
      cap_do <= ddr_do;
      cap_be <= ddr_be;

      cap_toggle <= ~cap_toggle;
    end
  end

  reg eng_done_toggle = 0;
  reg [1:0] ack_sync = 0;

  always @(posedge clk_sys) begin
    ack_sync <= {ack_sync[0], eng_done_toggle};
    ddr_ack  <= ack_sync[1];
  end

  ////////////////////////////////////////////////////////////////////////////////////////
  // SNI access (clk_mem)

  reg [2:0] cap_toggle_sync = 0;

  always @(posedge clk_mem) begin
    cap_toggle_sync <= {cap_toggle_sync[1:0], cap_toggle};
  end

  wire eng_pending = cap_toggle_sync[2] != eng_done_toggle;

  reg host_pending = 0;
  reg h_we = 0;
  reg [19:1] h_addr = 0;
  reg [15:0] h_din = 0;

  localparam IDLE = 0;
  localparam ENG_WORD = 1;
  localparam ENG_NEXT = 2;
  localparam WAIT_CLEAR = 3;
  localparam WAIT_READY = 4;

  reg [2:0] state = IDLE;
  reg is_eng = 0;
  reg [1:0] word_idx = 0;

  wire [1:0] word_be = cap_be[word_idx*2+:2];

  always @(posedge clk_mem) begin
    sni_rd_req <= 0;
    sni_wr_req <= 0;
    host_ack   <= 0;

    if (host_req) begin
      host_pending <= 1;

      h_we <= host_we;
      h_addr <= host_addr;
      h_din <= host_din;
    end

    case (state)
      IDLE: begin
        if (eng_pending) begin
          is_eng <= 1;
          word_idx <= 0;

          state <= ENG_WORD;
        end else if (host_pending) begin
          is_eng <= 0;

          sni_addr <= {1'b1, 4'b0, h_addr, 1'b0};
          sni_din <= h_din;
          sni_word <= 1;

          if (h_we) sni_wr_req <= 1;
          else sni_rd_req <= 1;

          state <= WAIT_CLEAR;
        end
      end
      ENG_WORD: begin
        if (cap_we && word_be == 2'b00) begin
          // Nothing to write in this word (header writes skip the count)
          state <= ENG_NEXT;
        end else begin
          sni_addr <= {1'b1, 2'b0, cap_addr, word_idx, 1'b0};
          sni_din  <= cap_do[word_idx*16+:16];
          sni_word <= 1;

          if (cap_we) begin
            // SDRAM byte writes take their data from din[7:0] and pick the lane from addr[0]
            case (word_be)
              2'b01: sni_word <= 0;
              2'b10: begin
                sni_word <= 0;
                sni_addr[0] <= 1;
                sni_din <= {8'h00, cap_do[word_idx*16+8+:8]};
              end
              default: ;
            endcase

            sni_wr_req <= 1;
          end else begin
            sni_rd_req <= 1;
          end

          state <= WAIT_CLEAR;
        end
      end
      ENG_NEXT: begin
        if (word_idx == 2'd3) begin
          eng_done_toggle <= ~eng_done_toggle;

          dbg_eng_ops <= dbg_eng_ops + 32'h1;
          if (cap_we) begin
            dbg_eng_writes <= dbg_eng_writes + 32'h1;
            if (cap_addr == 0) dbg_hdr0 <= cap_do;
          end
          if (cap_addr == 19'h1) dbg_hdr8 <= cap_we ? cap_do : ddr_di;

          state <= IDLE;
        end else begin
          word_idx <= word_idx + 2'd1;

          state <= ENG_WORD;
        end
      end
      WAIT_CLEAR: begin
        // SDRAM drops sni_ready the cycle after it sees the request edge
        state <= WAIT_READY;
      end
      WAIT_READY: begin
        if (sni_ready) begin
          if (is_eng) begin
            if (~cap_we) ddr_di[word_idx*16+:16] <= sni_dout;

            state <= ENG_NEXT;
          end else begin
            host_dout <= sni_dout;
            host_ack <= 1;
            host_pending <= 0;

            state <= IDLE;
          end
        end
      end
      default: state <= IDLE;
    endcase
  end
endmodule
