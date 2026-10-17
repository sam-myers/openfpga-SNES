// APF save state controller
//
// Connects APF's savestate start/load handshake and bridge transfers to the upstream savestate engine.
// The engine snapshots into an SDRAM buffer (see ss_buffer.sv); this module moves that buffer to and
// from APF through the bridge at 0x4xxxxxxx.
//
// Save: APF start -> pulse ss_save -> engine runs its routine on the next NMI (ss_busy high) ->
//   prefetch the buffer into a FIFO -> report ok -> APF reads the buffer sequentially.
// Load: APF writes the buffer through the bridge (queued in a FIFO and drained into SDRAM) -> APF load ->
//   pulse ss_load -> engine restores on the next NMI -> report ok.
//
// Byte N of the save state file is byte N of the buffer. The bridge carries byte N in bits [31:24].
module save_state_controller #(
    // Bytes transferred to/from APF. A full snapshot (WRAM, VRAM, ARAM, 128KB BSRAM, registers) is under
    // 400KB
    parameter SIZE = 32'h80000
) (
    input wire clk_74a,
    input wire clk_mem,

    // APF bridge (clk_74a)
    input wire bridge_wr,
    input wire bridge_rd,
    input wire [31:0] bridge_addr,
    input wire [31:0] bridge_wr_data,
    output wire [31:0] save_state_bridge_read_data,

    // APF save states (clk_74a)
    input  wire savestate_load,
    output wire savestate_load_ack_s,
    output wire savestate_load_busy_s,
    output wire savestate_load_ok_s,
    output wire savestate_load_err_s,

    input  wire savestate_start,
    output wire savestate_start_ack_s,
    output wire savestate_start_busy_s,
    output wire savestate_start_ok_s,
    output wire savestate_start_err_s,

    // Engine control (clk_mem, synchronized in MAIN_SNES)
    output reg ss_save = 0,
    output reg ss_load = 0,
    // clk_sys, synchronized here
    input wire ss_busy,
    input wire ss_supported,

    // Buffer host port (clk_mem)
    output reg host_req = 0,
    output reg host_we = 0,
    output reg [19:1] host_addr = 0,
    output reg [15:0] host_din = 0,
    input wire [15:0] host_dout,
    input wire host_ack
);
  localparam [17:0] SIZE_WORDS = SIZE / 4;

  // ~1.5s at 85.9MHz. Saves and loads wait for the game's next NMI/IRQ, which is normally within a frame
  localparam [27:0] TIMEOUT = 28'd128_000_000;

  // Number of 32-bit words prefetched before reporting a save as complete
  localparam PREFILL_WORDS = 256;

  ////////////////////////////////////////////////////////////////////////////////////////
  // Synchronizers

  reg [2:0] start_sync = 0;
  reg [2:0] load_sync = 0;
  reg [1:0] busy_sync = 0;
  reg [1:0] supported_sync = 0;

  always @(posedge clk_mem) begin
    start_sync <= {start_sync[1:0], savestate_start};
    load_sync <= {load_sync[1:0], savestate_load};
    busy_sync <= {busy_sync[0], ss_busy};
    supported_sync <= {supported_sync[0], ss_supported};
  end

  wire start_s = start_sync[2];
  wire load_s = load_sync[2];
  wire busy_s = busy_sync[1];
  wire supported_s = supported_sync[1];

  reg savestate_load_ack = 0;
  reg savestate_load_busy = 0;
  reg savestate_load_ok = 0;
  reg savestate_load_err = 0;

  reg savestate_start_ack = 0;
  reg savestate_start_busy = 0;
  reg savestate_start_ok = 0;
  reg savestate_start_err = 0;

  synch_3 #(
      .WIDTH(8)
  ) savestate_out (
      {
        savestate_load_ack,
        savestate_load_busy,
        savestate_load_ok,
        savestate_load_err,
        savestate_start_ack,
        savestate_start_busy,
        savestate_start_ok,
        savestate_start_err
      },
      {
        savestate_load_ack_s,
        savestate_load_busy_s,
        savestate_load_ok_s,
        savestate_load_err_s,
        savestate_start_ack_s,
        savestate_start_busy_s,
        savestate_start_ok_s,
        savestate_start_err_s
      },
      clk_74a
  );

  ////////////////////////////////////////////////////////////////////////////////////////
  // APF -> buffer (load). Bridge writes are queued with their word index

  wire bridge_ss_addr = bridge_addr[31:28] == 4'h4;

  reg prev_bridge_wr = 0;
  reg wr_fifo_wrreq = 0;
  reg [49:0] wr_fifo_data = 0;
  wire wr_fifo_full;
  // Sticky until the next load begins; a dropped word means a corrupt state
  reg wr_overflow = 0;
  reg [2:0] wr_overflow_clear_sync = 0;
  reg wr_overflow_clear = 0;

  always @(posedge clk_74a) begin
    prev_bridge_wr <= bridge_wr;
    wr_fifo_wrreq <= 0;

    wr_overflow_clear_sync <= {wr_overflow_clear_sync[1:0], wr_overflow_clear};

    if (~prev_bridge_wr && bridge_wr && bridge_ss_addr) begin
      wr_fifo_data <= {bridge_addr[19:2], bridge_wr_data};

      if (wr_fifo_full) wr_overflow <= 1;
      else wr_fifo_wrreq <= 1;
    end

    if (wr_overflow_clear_sync[2]) wr_overflow <= 0;
  end

  wire wr_fifo_empty;
  wire [49:0] wr_fifo_q;
  reg wr_fifo_rdreq = 0;

  dcfifo wr_fifo (
      .data(wr_fifo_data),
      .rdclk(clk_mem),
      .rdreq(wr_fifo_rdreq),
      .wrclk(clk_74a),
      .wrreq(wr_fifo_wrreq),
      .q(wr_fifo_q),
      .rdempty(wr_fifo_empty),
      .wrfull(wr_fifo_full),
      .aclr(1'b0)
  );
  defparam wr_fifo.intended_device_family = "Cyclone V", wr_fifo.lpm_numwords = 256,
      wr_fifo.lpm_showahead = "ON", wr_fifo.lpm_type = "dcfifo", wr_fifo.lpm_width = 50,
      wr_fifo.lpm_widthu = 8, wr_fifo.overflow_checking = "ON", wr_fifo.rdsync_delaypipe = 5,
      wr_fifo.underflow_checking = "ON", wr_fifo.use_eab = "ON", wr_fifo.wrsync_delaypipe = 5;

  reg [1:0] wr_overflow_sync = 0;

  always @(posedge clk_mem) begin
    wr_overflow_sync <= {wr_overflow_sync[0], wr_overflow};
  end

  ////////////////////////////////////////////////////////////////////////////////////////
  // Buffer -> APF (save). The buffer is prefetched sequentially into a FIFO ahead of APF's reads

  reg rd_fifo_wrreq = 0;
  reg [31:0] rd_fifo_data = 0;
  reg rd_fifo_clr = 0;
  wire [8:0] rd_fifo_wrusedw;
  wire rd_fifo_wrfull;

  wire rd_fifo_empty;
  wire [31:0] rd_fifo_q;
  reg rd_fifo_rdreq = 0;

  dcfifo rd_fifo (
      .data(rd_fifo_data),
      .rdclk(clk_74a),
      .rdreq(rd_fifo_rdreq),
      .wrclk(clk_mem),
      .wrreq(rd_fifo_wrreq),
      .q(rd_fifo_q),
      .rdempty(rd_fifo_empty),
      .wrusedw(rd_fifo_wrusedw),
      .wrfull(rd_fifo_wrfull),
      .aclr(rd_fifo_clr)
  );
  defparam rd_fifo.intended_device_family = "Cyclone V", rd_fifo.lpm_numwords = 512,
      rd_fifo.lpm_showahead = "ON", rd_fifo.lpm_type = "dcfifo", rd_fifo.lpm_width = 32,
      rd_fifo.lpm_widthu = 9, rd_fifo.overflow_checking = "ON", rd_fifo.rdsync_delaypipe = 5,
      rd_fifo.underflow_checking = "ON", rd_fifo.use_eab = "ON", rd_fifo.wrsync_delaypipe = 5,
      rd_fifo.read_aclr_synch = "ON", rd_fifo.write_aclr_synch = "ON";

  // Toggled each time the prefetch restarts, so the APF side forgets its read position
  reg rd_epoch = 0;
  reg [2:0] rd_epoch_sync = 0;
  reg prev_rd_epoch = 0;

  reg prev_bridge_rd = 0;
  reg [17:0] cur_idx = 18'h3FFFF;
  reg [31:0] cur_data = 0;

  wire [17:0] bridge_idx = bridge_addr[19:2];

  // APF may read the same word more than once, so hold the last popped word
  assign save_state_bridge_read_data = bridge_idx == cur_idx ? cur_data : rd_fifo_q;

  always @(posedge clk_74a) begin
    prev_bridge_rd <= bridge_rd;
    rd_fifo_rdreq <= 0;

    rd_epoch_sync <= {rd_epoch_sync[1:0], rd_epoch};
    prev_rd_epoch <= rd_epoch_sync[2];

    if (prev_rd_epoch != rd_epoch_sync[2]) begin
      cur_idx <= 18'h3FFFF;
    end else if (~prev_bridge_rd && bridge_rd && bridge_ss_addr) begin
      if (bridge_idx == cur_idx + 18'h1 && ~rd_fifo_empty) begin
        cur_idx <= bridge_idx;
        cur_data <= rd_fifo_q;
        rd_fifo_rdreq <= 1;
      end
    end
  end

  ////////////////////////////////////////////////////////////////////////////////////////
  // Buffer access sequencer (clk_mem). Write draining takes priority over prefetching

  localparam SEQ_IDLE = 0;
  localparam SEQ_WRITE_LO = 1;
  localparam SEQ_WRITE_HI = 2;
  localparam SEQ_READ_LO = 3;
  localparam SEQ_READ_HI = 4;
  localparam SEQ_PUSH = 5;
  localparam SEQ_WAIT = 6;

  reg [2:0] seq_state = SEQ_IDLE;
  reg [2:0] seq_return = SEQ_IDLE;

  reg [49:0] wr_entry = 0;
  reg [15:0] rd_lo = 0;

  reg prefetch_en = 0;
  reg [17:0] pf_idx = 0;

  wire seq_idle = seq_state == SEQ_IDLE;

  task issue(input we, input [19:1] addr, input [15:0] data, input [2:0] ret);
    begin
      host_req <= 1;
      host_we <= we;
      host_addr <= addr;
      host_din <= data;
      seq_return <= ret;
      seq_state <= SEQ_WAIT;
    end
  endtask

  // Pulsed by the APF handshake FSM
  reg prefetch_restart = 0;
  reg prefetch_stop = 0;

  always @(posedge clk_mem) begin
    host_req <= 0;
    wr_fifo_rdreq <= 0;
    rd_fifo_wrreq <= 0;

    if (prefetch_restart) begin
      prefetch_en <= 1;
      pf_idx <= 0;
    end

    if (prefetch_stop) prefetch_en <= 0;

    // Any incoming load data invalidates the prefetched save
    if (~wr_fifo_empty) prefetch_en <= 0;

    case (seq_state)
      SEQ_IDLE: begin
        if (~wr_fifo_empty) begin
          wr_entry <= wr_fifo_q;
          wr_fifo_rdreq <= 1;

          seq_state <= SEQ_WRITE_LO;
        end else if (prefetch_en && ~prefetch_restart && pf_idx != SIZE_WORDS &&
            rd_fifo_wrusedw < 9'd500 && ~rd_fifo_wrfull) begin
          seq_state <= SEQ_READ_LO;
        end
      end
      SEQ_WRITE_LO: issue(1, {wr_entry[49:32], 1'b0}, {wr_entry[23:16], wr_entry[31:24]}, SEQ_WRITE_HI);
      SEQ_WRITE_HI: issue(1, {wr_entry[49:32], 1'b1}, {wr_entry[7:0], wr_entry[15:8]}, SEQ_IDLE);
      SEQ_READ_LO: issue(0, {pf_idx, 1'b0}, 16'h0, SEQ_READ_HI);
      SEQ_READ_HI: begin
        rd_lo <= host_dout;
        issue(0, {pf_idx, 1'b1}, 16'h0, SEQ_PUSH);
      end
      SEQ_PUSH: begin
        rd_fifo_data <= {rd_lo[7:0], rd_lo[15:8], host_dout[7:0], host_dout[15:8]};
        rd_fifo_wrreq <= 1;
        pf_idx <= pf_idx + 18'h1;

        seq_state <= SEQ_IDLE;
      end
      SEQ_WAIT: begin
        if (host_ack) seq_state <= seq_return;
      end
      default: seq_state <= SEQ_IDLE;
    endcase
  end

  ////////////////////////////////////////////////////////////////////////////////////////
  // APF handshake FSM (clk_mem)

  localparam NONE = 0;
  localparam SAVE_WAIT_START = 1;
  localparam SAVE_WAIT_END = 2;
  localparam SAVE_SETTLE = 3;
  localparam SAVE_PREFILL = 4;
  localparam LOAD_DRAIN = 5;
  localparam LOAD_WAIT_START = 6;
  localparam LOAD_WAIT_END = 7;

  reg [3:0] state = NONE;

  reg prev_start_s = 0;
  reg prev_load_s = 0;
  reg [27:0] timer = 0;

  always @(posedge clk_mem) begin
    prev_start_s <= start_s;
    prev_load_s <= load_s;

    prefetch_restart <= 0;
    prefetch_stop <= 0;
    rd_fifo_clr <= 0;
    wr_overflow_clear <= 0;

    timer <= timer + 28'h1;

    // Hold acks until APF drops its request
    if (~start_s) savestate_start_ack <= 0;
    if (~load_s) savestate_load_ack <= 0;

    if (start_s && ~prev_start_s) begin
      savestate_start_ack <= 1;
      savestate_start_busy <= 1;
      savestate_start_ok <= 0;
      savestate_start_err <= 0;

      savestate_load_ok <= 0;
      savestate_load_err <= 0;

      prefetch_stop <= 1;
      ss_save <= 0;
      ss_load <= 0;
      timer <= 0;

      if (supported_s) begin
        ss_save <= 1;
        state <= SAVE_WAIT_START;
      end else begin
        savestate_start_busy <= 0;
        savestate_start_err <= 1;
        state <= NONE;
      end
    end else if (load_s && ~prev_load_s) begin
      savestate_load_ack <= 1;
      savestate_load_busy <= 1;
      savestate_load_ok <= 0;
      savestate_load_err <= 0;

      savestate_start_ok <= 0;
      savestate_start_err <= 0;

      ss_save <= 0;
      ss_load <= 0;
      timer <= 0;

      state <= LOAD_DRAIN;
    end else begin
      case (state)
        SAVE_WAIT_START: begin
          if (busy_s) begin
            // Engine has latched the request
            ss_save <= 0;
            timer <= 0;
            state <= SAVE_WAIT_END;
          end else if (timer == TIMEOUT) begin
            ss_save <= 0;
            savestate_start_busy <= 0;
            savestate_start_err <= 1;
            state <= NONE;
          end
        end
        SAVE_WAIT_END: begin
          if (~busy_s) begin
            // Let the final header write reach SDRAM before reading the buffer back
            timer <= 0;
            rd_fifo_clr <= 1;
            state <= SAVE_SETTLE;
          end else if (timer == TIMEOUT) begin
            savestate_start_busy <= 0;
            savestate_start_err <= 1;
            state <= NONE;
          end
        end
        SAVE_SETTLE: begin
          if (timer < 28'd16) rd_fifo_clr <= 1;

          if (timer == 28'd4096) begin
            rd_epoch <= ~rd_epoch;
            prefetch_restart <= 1;
            state <= SAVE_PREFILL;
          end
        end
        SAVE_PREFILL: begin
          if (rd_fifo_wrusedw >= PREFILL_WORDS || (prefetch_en && pf_idx == SIZE_WORDS)) begin
            savestate_start_busy <= 0;
            savestate_start_ok <= 1;
            state <= NONE;
          end else if (~prefetch_en && ~prefetch_restart) begin
            // Load data arrived mid-save
            savestate_start_busy <= 0;
            savestate_start_err <= 1;
            state <= NONE;
          end
        end
        LOAD_DRAIN: begin
          if (wr_fifo_empty && seq_idle) begin
            timer <= 0;

            if (~supported_s || wr_overflow_sync[1]) begin
              wr_overflow_clear <= 1;
              savestate_load_busy <= 0;
              savestate_load_err <= 1;
              state <= NONE;
            end else begin
              ss_load <= 1;
              state <= LOAD_WAIT_START;
            end
          end
        end
        LOAD_WAIT_START: begin
          if (busy_s) begin
            ss_load <= 0;
            timer <= 0;
            state <= LOAD_WAIT_END;
          end else if (timer == TIMEOUT) begin
            // Engine rejected the state (bad header) or the game never hit NMI/IRQ
            ss_load <= 0;
            savestate_load_busy <= 0;
            savestate_load_err <= 1;
            state <= NONE;
          end
        end
        LOAD_WAIT_END: begin
          if (~busy_s) begin
            savestate_load_busy <= 0;
            savestate_load_ok <= 1;
            state <= NONE;
          end else if (timer == TIMEOUT) begin
            savestate_load_busy <= 0;
            savestate_load_err <= 1;
            state <= NONE;
          end
        end
        default: ;
      endcase
    end
  end
endmodule
