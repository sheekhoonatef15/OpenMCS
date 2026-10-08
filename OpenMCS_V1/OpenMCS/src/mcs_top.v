// OpenMCS Production V3.3 -- trigger-anchored live histogram streaming
//
// The tested V2.3 synchronizers, dwell timing, UART protocol, and single-port
// Up to 16,384 x 32-bit accumulated histogram BSRAM is provided. Completed bins are
// queued in a 16-entry register FIFO, decoupling acquisition boundaries from
// histogram read/modify/write latency. Bin changes occur on exact 100 MHz
// boundaries. Triggers received during an active pass are ignored and are
// never queued or deferred; the next pass requires a new trigger while armed.
// The FIFO head is explicitly registered and dwell timing uses a down-counter
// so the two main acquisition-domain critical paths remain short at 100 MHz.

module mcs_step8 #(
    parameter integer ACQ_CLOCK_HZ = 100_000_000,
    parameter integer CTRL_CLOCK_HZ = 50_000_000,
    parameter integer MAX_BINS = 16_384,
    parameter integer DEFAULT_BINS = 10_000,
    parameter integer DEFAULT_DWELL_CYCLES = 30,
    parameter integer UART_BAUD = 5_000_000
) (
    input wire clk_50m,
    input wire event_in,
    input wire trigger_in,
    input wire uart_rx,
    output wire uart_tx,
    output wire led0_n,
    output wire led1_n,

    // Hardware pass/stop sidecar. These ports do not gate the acquisition
    // datapath. pass_toggle_out exposes the existing pass-snapshot toggle;
    // stop_request_in is translated in the 50 MHz control domain into the
    // exact same stop-command toggle already used by UART "P".
    output wire pass_toggle_out,
    input  wire accept_enable_in,
    input  wire stop_request_in,
    output wire stop_ack_out
);
    localparam integer HIST_ADDR_WIDTH = $clog2(MAX_BINS);
    localparam integer BIN_FIFO_DEPTH = 16;
    localparam integer BIN_FIFO_PTR_WIDTH = $clog2(BIN_FIFO_DEPTH);
    localparam integer LIVE_BLOCK_BINS = 256;
    // Below 30 us/bin, an ordinary 16,384-bin MCS6 frame already refreshes in
    // about 131 ms and avoids needless partial-frame traffic.
    localparam [31:0] LIVE_MIN_DWELL_CYCLES = (ACQ_CLOCK_HZ / 1_000_000) * 30;
    localparam [1:0] ACC_IDLE = 2'd0,
                     ACC_CAPTURE = 2'd1,
                     ACC_WRITE = 2'd2;
    localparam [1:0] MCS_STOPPED = 2'd0,
                     MCS_ARMED   = 2'd1,
                     MCS_ACTIVE  = 2'd2;
    localparam [1:0] RAM_READ_NONE = 2'd0,
                     RAM_READ_UART = 2'd2;
    localparam [3:0] ST_IDLE       = 4'd0,
                     ST_ARMED      = 4'd1,
                     ST_TX_HEADER  = 4'd2,
                     ST_HIST_WAIT  = 4'd3,
                     ST_HIST_LATCH = 4'd4,
                     ST_TX_DATA    = 4'd5,
                     ST_TX_TOTAL   = 4'd6,
                     ST_TX_TRAILER = 4'd7,
                     ST_CFG_ACK    = 4'd8,
                     ST_CLEAR_WAIT = 4'd9;
    localparam [31:0] ACQ_CLOCK_HZ_U32 = ACQ_CLOCK_HZ;

    function [7:0] crc8_next;
        input [7:0] crc;
        input [7:0] data;
        integer i;
        reg [7:0] c;
        begin
            c = crc ^ data;
            for (i = 0; i < 8; i = i + 1) begin
                if (c[7]) c = (c << 1) ^ 8'h07;
                else c = c << 1;
            end
            crc8_next = c;
        end
    endfunction

    function [7:0] ack_crc_value;
        input [7:0] status;
        input [15:0] bins;
        input [31:0] dwell;
        reg [7:0] c;
        begin
            c = 8'h00;
            c = crc8_next(c, "A"); c = crc8_next(c, "C");
            c = crc8_next(c, "K"); c = crc8_next(c, "5");
            c = crc8_next(c, status);
            c = crc8_next(c, bins[7:0]); c = crc8_next(c, bins[15:8]);
            c = crc8_next(c, dwell[7:0]); c = crc8_next(c, dwell[15:8]);
            c = crc8_next(c, dwell[23:16]); c = crc8_next(c, dwell[31:24]);
            ack_crc_value = c;
        end
    endfunction

    wire clk_150m, clk_300m, pll_lock;
    gowin_pll_150 clock_pll (
        .clkin(clk_50m), .clkout_0(clk_150m),
        .clkout_300(clk_300m), .lock(pll_lock)
    );
    assign led0_n = ~pll_lock;

    // ----------------------- configuration UART ------------------------
    // CFG5/ACK5 is retained for compatibility. A successful CFG5 also clears
    // the FPGA accumulated histogram and its pass counter.
    wire [7:0] rx_data;
    wire rx_valid, rx_framing_error;
    uart_rx_8n1 #(.CLOCK_HZ(CTRL_CLOCK_HZ), .BAUD(UART_BAUD)) uart_receiver (
        .clk(clk_50m), .rx(uart_rx), .data(rx_data),
        .valid(rx_valid), .framing_error(rx_framing_error)
    );

    reg [3:0] cfg_parse_index = 4'd0;
    reg [7:0] cfg_crc = 8'h00;
    reg [15:0] cfg_candidate_bins = DEFAULT_BINS;
    reg [31:0] cfg_candidate_dwell = DEFAULT_DWELL_CYCLES;
    reg cfg_request_toggle = 1'b0;
    reg cfg_checksum_ok = 1'b0;
    // Single-byte run control from the Teensy. Toggles are used so each
    // command can be transferred safely into the 100 MHz acquisition domain.
    reg run_command_toggle_27 = 1'b0;
    reg counter_command_toggle_27 = 1'b0;
    reg stop_command_toggle_27 = 1'b0;
    // Absolute accumulated-pass target. Zero means unlimited. Reissuing START
    // resumes toward this target instead of adding a fresh pass quota.
    reg [31:0] pass_limit_27 = 32'd0;
    reg explicit_clear_toggle_27 = 1'b0;
    reg cfg_stop_request_27 = 1'b0;
    // MCS H2 trigger polarity: 0 = rising (U), 1 = falling (D).
    // This setting changes only the FPGA MCS trigger edge; it does not alter H2/H4.
    reg trigger_falling_27 = 1'b0;

    // Hardware STOP input synchronizer/edge detector.  This lives entirely
    // in the existing 50 MHz control domain so the 100 MHz stop path below
    // remains unchanged. A rising edge produces the same toggle as UART "P".
    reg stop_hw_meta_27 = 1'b0;
    reg stop_hw_sync_27 = 1'b0;
    reg stop_hw_seen_27 = 1'b0;

    always @(posedge clk_50m) begin
        stop_hw_meta_27 <= stop_request_in;
        stop_hw_sync_27 <= stop_hw_meta_27;
        stop_hw_seen_27 <= stop_hw_sync_27;

        if (cfg_stop_request_27) begin
            stop_command_toggle_27 <= ~stop_command_toggle_27;
        end
        if (stop_hw_sync_27 && !stop_hw_seen_27) begin
            stop_command_toggle_27 <= ~stop_command_toggle_27;
        end

        if (rx_framing_error) begin
            cfg_parse_index <= 4'd0;
            cfg_crc <= 8'h00;
        end else if (rx_valid) begin
            case (cfg_parse_index)
                4'd0: begin
                    if (rx_data == "C") begin
                        cfg_parse_index <= 4'd1;
                        cfg_crc <= crc8_next(8'h00, rx_data);
                    end else if (rx_data == "L") begin
                        cfg_parse_index <= 4'd11;
                    end else if (rx_data == "R") begin
                        run_command_toggle_27 <= ~run_command_toggle_27;
                    end else if (rx_data == "K") begin
                        counter_command_toggle_27 <= ~counter_command_toggle_27;
                    end else if (rx_data == "P") begin
                        stop_command_toggle_27 <= ~stop_command_toggle_27;
                    end else if (rx_data == "X") begin
                        // Dedicated, geometry-preserving histogram clear.
                        explicit_clear_toggle_27 <= ~explicit_clear_toggle_27;
                    end else if (rx_data == "U") begin
                        trigger_falling_27 <= 1'b0;
                    end else if (rx_data == "D") begin
                        trigger_falling_27 <= 1'b1;
                    end
                end
                4'd1: begin
                    if (rx_data == "F") begin
                        cfg_parse_index <= 4'd2;
                        cfg_crc <= crc8_next(cfg_crc, rx_data);
                    end else begin
                        cfg_parse_index <= (rx_data == "C") ? 4'd1 : 4'd0;
                        cfg_crc <= (rx_data == "C") ?
                                   crc8_next(8'h00, rx_data) : 8'h00;
                    end
                end
                4'd2: begin
                    if (rx_data == "G") begin
                        cfg_parse_index <= 4'd3;
                        cfg_crc <= crc8_next(cfg_crc, rx_data);
                    end else begin
                        cfg_parse_index <= (rx_data == "C") ? 4'd1 : 4'd0;
                        cfg_crc <= (rx_data == "C") ?
                                   crc8_next(8'h00, rx_data) : 8'h00;
                    end
                end
                4'd3: begin
                    if (rx_data == "5") begin
                        cfg_parse_index <= 4'd4;
                        cfg_crc <= crc8_next(cfg_crc, rx_data);
                    end else begin
                        cfg_parse_index <= (rx_data == "C") ? 4'd1 : 4'd0;
                        cfg_crc <= (rx_data == "C") ?
                                   crc8_next(8'h00, rx_data) : 8'h00;
                    end
                end
                4'd4: begin
                    cfg_candidate_bins[7:0] <= rx_data;
                    cfg_crc <= crc8_next(cfg_crc, rx_data);
                    cfg_parse_index <= 4'd5;
                end
                4'd5: begin
                    cfg_candidate_bins[15:8] <= rx_data;
                    cfg_crc <= crc8_next(cfg_crc, rx_data);
                    cfg_parse_index <= 4'd6;
                end
                4'd6: begin
                    cfg_candidate_dwell[7:0] <= rx_data;
                    cfg_crc <= crc8_next(cfg_crc, rx_data);
                    cfg_parse_index <= 4'd7;
                end
                4'd7: begin
                    cfg_candidate_dwell[15:8] <= rx_data;
                    cfg_crc <= crc8_next(cfg_crc, rx_data);
                    cfg_parse_index <= 4'd8;
                end
                4'd8: begin
                    cfg_candidate_dwell[23:16] <= rx_data;
                    cfg_crc <= crc8_next(cfg_crc, rx_data);
                    cfg_parse_index <= 4'd9;
                end
                4'd9: begin
                    cfg_candidate_dwell[31:24] <= rx_data;
                    cfg_crc <= crc8_next(cfg_crc, rx_data);
                    cfg_parse_index <= 4'd10;
                end
                4'd11: begin
                    pass_limit_27[7:0] <= rx_data;
                    cfg_parse_index <= 4'd12;
                end
                4'd12: begin
                    pass_limit_27[15:8] <= rx_data;
                    cfg_parse_index <= 4'd13;
                end
                4'd13: begin
                    pass_limit_27[23:16] <= rx_data;
                    cfg_parse_index <= 4'd14;
                end
                4'd14: begin
                    pass_limit_27[31:24] <= rx_data;
                    cfg_parse_index <= 4'd0;
                end
                default: begin
                    cfg_checksum_ok <= (rx_data == cfg_crc);
                    cfg_request_toggle <= ~cfg_request_toggle;
                    cfg_parse_index <= 4'd0;
                    cfg_crc <= 8'h00;
                end
            endcase
        end
    end

    // ---------------- cumulative-counter acquisition domain -------------
    // Experimental Arkani-inspired architecture. The physical counters below
    // are free-running and are never reset by START, STOP, CLEAR, bin rollover,
    // pass rollover, histogram service, or UART activity.
    // Continuously running physical counters are 32 bits wide. Every consumer
    // takes an interval as a modulo-2^32 difference, so a physical wrap never
    // changes a result, and the histogram bins and UART protocol are 32 bits
    // anyway. A 64-bit version of these counters produced rare, random
    // single-bin corruption on hardware (the 32-bit V3 architecture did not),
    // so do NOT widen them without re-validating on hardware.
    localparam integer CUM_WIDTH = 32;
    localparam integer SNAP_FIFO_DEPTH = 32;
    localparam integer SNAP_FIFO_PTR_WIDTH = $clog2(SNAP_FIFO_DEPTH);

    // Return only the number of event rises preceding a selected fine phase.
    // Keeping this function two bits wide avoids the shared 64-bit mux used by
    // the rejected boundary-function implementation.
    function [1:0] boundary_phase_increment;
        input [1:0] phase;
        input rise_q0;
        input rise_q1;
        begin
            case (phase)
                2'd0:    boundary_phase_increment = 2'd0;
                2'd1:    boundary_phase_increment = {1'b0, rise_q0};
                default: boundary_phase_increment =
                    {1'b0, rise_q0} + {1'b0, rise_q1};
            endcase
        end
    endfunction

    reg [2:0] acq_lock_pipe = 3'b000;
    always @(posedge clk_150m)
        acq_lock_pipe <= {acq_lock_pipe[1:0], pll_lock};
    wire acq_clock_ready = &acq_lock_pipe;

    // Single-domain 300 MHz input synchronizers. CLKOUT1 (300 MHz) and CLKOUT0
    // (100 MHz) come from the same PLL VCO. The fast sampler deliberately uses
    // the FALLING edge: its registers therefore never launch on the aligned
    // 100 MHz rising edge that captures the completed three-sample tuple.
    // This removes V7's same-edge multi-bit transfer, which could capture a
    // mixture of old/new shift-register bits after routing.
    // asynchronous input passes a conventional two-register synchronizer, then
    // a 3-deep sample shift register in the SAME 300 MHz domain. Every 100 MHz
    // edge captures three consecutive 3.333 ns samples. Chronological order:
    // phase 0 (oldest) -> phase 1 -> phase 2 (newest).
    // Request the dedicated pad-side input register for the first
    // metastability-catching stage. The second stage remains a separately
    // preserved fabric register at the matched constrained location.
    (* ASYNC_REG = "TRUE", syn_keep = 1, syn_preserve = 1, syn_useioff = 1 *)
    reg event_meta_300 = 1'b0;
    (* ASYNC_REG = "TRUE", syn_keep = 1, syn_preserve = 1 *)
    reg event_sync_300 = 1'b0;
    (* ASYNC_REG = "TRUE", syn_keep = 1, syn_preserve = 1, syn_useioff = 1 *)
    reg trigger_meta_300 = 1'b0;
    (* ASYNC_REG = "TRUE", syn_keep = 1, syn_preserve = 1 *)
    reg trigger_sync_300 = 1'b0;
    (* syn_keep = 1, syn_preserve = 1 *)
    reg [2:0] event_shift_300 = 3'b000;    // [0]=newest ... [2]=oldest
    (* syn_keep = 1, syn_preserve = 1 *)
    reg [2:0] trigger_shift_300 = 3'b000;

    always @(negedge clk_300m) begin
        event_meta_300   <= event_in;
        event_sync_300   <= event_meta_300;
        trigger_meta_300 <= trigger_in;
        trigger_sync_300 <= trigger_meta_300;
        event_shift_300   <= {event_shift_300[1:0], event_sync_300};
        trigger_shift_300 <= {trigger_shift_300[1:0], trigger_sync_300};
    end

    // Coherent two-stage fast-to-histogram transfer.
    //
    // At each 100 MHz FALLING edge, tuple_half captures the OLD value of the
    // 300 MHz falling-edge shift register.  Because a 300 MHz falling edge is
    // coincident with that 100 MHz falling edge, ordinary flip-flop semantics
    // intentionally select the tuple launched by the preceding 300 MHz falling
    // edge.  The setup relationship is therefore one full 300 MHz period
    // (3.333 ns), with the coincident new launch covered by the hold check.
    // tuple_half then remains unchanged for 5 ns before the 100 MHz rising-edge
    // acquisition registers consume it. This replaces the failed 1.667 ns
    // direct shift->sample paths without dropping or duplicating a sample.
    (* syn_keep = 1, syn_preserve = 1 *)
    reg [2:0] event_tuple_half = 3'b000;
    (* syn_keep = 1, syn_preserve = 1 *)
    reg [2:0] trigger_tuple_half = 3'b000;

    always @(negedge clk_150m) begin
        event_tuple_half   <= {event_shift_300[0], event_shift_300[1], event_shift_300[2]};
        trigger_tuple_half <= {trigger_shift_300[0], trigger_shift_300[1], trigger_shift_300[2]};
    end

    (* syn_keep = 1, syn_preserve = 1 *)
    reg [2:0] event_sample = 3'b000;       // [0]=phase0 (oldest) ... [2]=phase2
    (* syn_keep = 1, syn_preserve = 1 *)
    reg [2:0] trigger_sample = 3'b000;
    reg event_previous_2 = 1'b0;
    reg trigger_previous_2 = 1'b0;
    reg accept_meta_135 = 1'b0, accept_sync_135 = 1'b0;
    reg accepted_pass_toggle_135 = 1'b0;

    always @(posedge clk_150m) begin
        event_sample   <= event_tuple_half;
        trigger_sample <= trigger_tuple_half;
        event_previous_2   <= event_sample[2];
        trigger_previous_2 <= trigger_sample[2];
        accept_meta_135 <= accept_enable_in;
        accept_sync_135 <= accept_meta_135;
    end

    // Edge decode on one glitch-free stream. Order: previous ph2 -> ph0 -> ph1 -> ph2.
    wire event_rise_q0 = ~event_previous_2 & event_sample[0];
    wire event_rise_q1 = ~event_sample[0] & event_sample[1];
    wire event_rise_q2 = ~event_sample[1] & event_sample[2];
    wire event_rise = event_rise_q0 | event_rise_q1 | event_rise_q2;

    wire trigger_rise_q0 = ~trigger_previous_2 & trigger_sample[0];
    wire trigger_rise_q1 = ~trigger_sample[0] & trigger_sample[1];
    wire trigger_rise_q2 = ~trigger_sample[1] & trigger_sample[2];
    wire trigger_rise = trigger_rise_q0 | trigger_rise_q1 | trigger_rise_q2;

    wire trigger_fall_q0 = trigger_previous_2 & ~trigger_sample[0];
    wire trigger_fall_q1 = trigger_sample[0] & ~trigger_sample[1];
    wire trigger_fall_q2 = trigger_sample[1] & ~trigger_sample[2];
    wire trigger_fall = trigger_fall_q0 | trigger_fall_q1 | trigger_fall_q2;

    // Phase (0,1,2 = 0, 3.333, 6.667 ns) of the FIRST qualified transition.
    wire [1:0] trigger_rise_phase = trigger_rise_q0 ? 2'd0 :
                                    trigger_rise_q1 ? 2'd1 : 2'd2;
    wire [1:0] trigger_fall_phase = trigger_fall_q0 ? 2'd0 :
                                    trigger_fall_q1 ? 2'd1 : 2'd2;

    reg trigger_falling_meta_135 = 1'b0;
    reg trigger_falling_sync_135 = 1'b0;
    wire mcs_trigger_edge = trigger_falling_sync_135 ? trigger_fall : trigger_rise;
    wire [1:0] mcs_trigger_phase = trigger_falling_sync_135 ?
                                   trigger_fall_phase : trigger_rise_phase;

    // Control/configuration crossing. Names are retained for UART compatibility.
    reg clear_toggle_27 = 1'b0;
    reg pass_ack_toggle_27 = 1'b0;
    reg uart_ram_request_toggle_27 = 1'b0;
    reg [HIST_ADDR_WIDTH-1:0] uart_ram_request_addr_27 = {HIST_ADDR_WIDTH{1'b0}};
    reg [15:0] armed_bins_27 = DEFAULT_BINS;
    reg [31:0] armed_dwell_27 = DEFAULT_DWELL_CYCLES;
    reg live_enable_27 = 1'b0;
    reg clear_meta_135 = 1'b0, clear_sync_135 = 1'b0;
    reg clear_arm_seen_135 = 1'b0;
    reg run_command_meta_135 = 1'b0, run_command_sync_135 = 1'b0;
    reg run_command_seen_135 = 1'b0;
    reg counter_command_meta_135 = 1'b0, counter_command_sync_135 = 1'b0;
    reg counter_command_seen_135 = 1'b0;
    reg stop_command_meta_135 = 1'b0, stop_command_sync_135 = 1'b0;
    reg stop_command_seen_135 = 1'b0;
    reg run_enabled_135 = 1'b0;
    reg counter_mode_135 = 1'b0;
    reg uart_req_meta_135 = 1'b0, uart_req_sync_135 = 1'b0;
    reg uart_req_safe_135 = 1'b0, uart_req_seen_135 = 1'b0;
    reg [HIST_ADDR_WIDTH-1:0] uart_addr_meta_135 = {HIST_ADDR_WIDTH{1'b0}};
    reg [HIST_ADDR_WIDTH-1:0] uart_addr_sync_135 = {HIST_ADDR_WIDTH{1'b0}};
    reg [15:0] bins_meta_135 = DEFAULT_BINS, bins_sync_135 = DEFAULT_BINS;
    reg [31:0] dwell_meta_135 = DEFAULT_DWELL_CYCLES;
    reg [31:0] dwell_sync_135 = DEFAULT_DWELL_CYCLES;
    reg live_enable_meta_135 = 1'b0, live_enable_sync_135 = 1'b0;
    reg [31:0] pass_limit_meta_135 = 32'd0, pass_limit_sync_135 = 32'd0;
    reg [31:0] passes_remaining_135 = 32'd0;
    reg quota_limited_135 = 1'b0;
    reg [31:0] pending_trigger_count_135 = 32'd0;
    reg trigger_queue_overflow_135 = 1'b0;
    reg [31:0] dwell_reload_135 = DEFAULT_DWELL_CYCLES;
    reg [HIST_ADDR_WIDTH-1:0] last_bin_135 = DEFAULT_BINS[HIST_ADDR_WIDTH-1:0] - 1'b1;
    reg [1:0] mcs_state_135 = MCS_STOPPED;
    wire armed_135 = (mcs_state_135 == MCS_ARMED);
    wire acquiring = (mcs_state_135 == MCS_ACTIVE);
    reg [1:0] active_trigger_phase_135 = 2'd0;

    // Physical counters: continuous modulo-2^32 counters. No measurement command
    // ever clears these registers. Modular subtraction recovers interval counts.
    reg [CUM_WIDTH-1:0] cumulative_event_counter_135 = {CUM_WIDTH{1'b0}};
    reg [CUM_WIDTH-1:0] cumulative_trigger_counter_135 = {CUM_WIDTH{1'b0}};

    // Cumulative prefixes for the three 3.333 ns boundaries in the current
    // tuple. cumulative_event_counter_135 holds all earlier tuples; prefix_qk
    // adds only the events in phases earlier than k.
    wire [CUM_WIDTH-1:0] event_prefix_q0 = cumulative_event_counter_135;
    wire [CUM_WIDTH-1:0] event_prefix_q1 = event_prefix_q0 + (event_rise_q0 ? 1'b1 : 1'b0);
    wire [CUM_WIDTH-1:0] event_prefix_q2 = event_prefix_q1 + (event_rise_q1 ? 1'b1 : 1'b0);
    wire [CUM_WIDTH-1:0] event_prefix_q3 = event_prefix_q2 + (event_rise_q2 ? 1'b1 : 1'b0);
    wire [1:0] active_boundary_increment = boundary_phase_increment(
        active_trigger_phase_135, event_rise_q0, event_rise_q1);
    wire [1:0] launch_boundary_increment = boundary_phase_increment(
        mcs_trigger_phase, event_rise_q0, event_rise_q1);
    wire [CUM_WIDTH-1:0] active_phase_event_prefix =
        event_prefix_q0 + active_boundary_increment;
    wire [CUM_WIDTH-1:0] launch_phase_event_prefix =
        event_prefix_q0 + launch_boundary_increment;
    always @(posedge clk_150m) begin
        if (acq_clock_ready) begin
            if (event_rise) begin
                cumulative_event_counter_135 <= event_prefix_q3;
            end
            if (trigger_rise) begin
                cumulative_trigger_counter_135 <= cumulative_trigger_counter_135 + 1'b1;
            end
        end
    end

    // Histogram BRAM remains exactly the conventional histogram expected by GUI.
    reg ram_enable_135;
    reg ram_write_enable_135;
    reg [HIST_ADDR_WIDTH-1:0] ram_address_135;
    reg [31:0] ram_write_data_135;
    wire [31:0] ram_read_data_135;
    reg [1:0] ram_read_tag_135 = RAM_READ_NONE;
    reg uart_read_pending_135 = 1'b0;
    reg [HIST_ADDR_WIDTH-1:0] uart_read_addr_135 = {HIST_ADDR_WIDTH{1'b0}};
    reg [31:0] uart_response_data_135 = 32'd0;
    reg uart_response_toggle_135 = 1'b0;

    // Snapshot FIFO. Acquisition only performs sequential register writes of
    // {address, cumulative snapshot, previous snapshot, last}. No histogram
    // read/modify/write is present in the real-time bin-boundary path.
    reg [HIST_ADDR_WIDTH-1:0] snap_fifo_addr [0:SNAP_FIFO_DEPTH-1];
    reg [CUM_WIDTH-1:0] snap_fifo_curr [0:SNAP_FIFO_DEPTH-1];
    reg [CUM_WIDTH-1:0] snap_fifo_prev [0:SNAP_FIFO_DEPTH-1];
    reg snap_fifo_last [0:SNAP_FIFO_DEPTH-1];
    reg [SNAP_FIFO_PTR_WIDTH-1:0] snap_wr_ptr = 0, snap_rd_ptr = 0;
    reg [SNAP_FIFO_PTR_WIDTH:0] snap_count = 0;
    reg snap_fifo_overflow_135 = 1'b0;
    wire snap_empty = (snap_count == 0);
    wire snap_full = (snap_count == SNAP_FIFO_DEPTH);

    // Background differencer + histogram accumulator. One entry requires three
    // clocks: dequeue/issue BRAM read, capture/add, write. Thus throughput is
    // one bin / 3 clocks, twice the required one bin / 6 clocks at 60 ns dwell.
    reg [1:0] accumulator_state = ACC_IDLE;
    reg [HIST_ADDR_WIDTH-1:0] accumulator_addr = 0;
    reg [CUM_WIDTH-1:0] accumulator_curr = 0, accumulator_prev = 0;
    reg accumulator_last = 0;
    reg [32:0] accumulator_sum = 0;
    wire [CUM_WIDTH-1:0] accumulator_delta_mod = accumulator_curr - accumulator_prev;
    // The histogram remains 32 bits, so the established accumulator consumes
    // the low 32 bits of each 64-bit interval delta.  Acquisition/FIFO/BRAM
    // scheduling is otherwise identical to MATCHED_ROUTE_V3.

    reg histogram_ready_135 = 1'b0;
    reg clear_active_135 = 1'b0;
    reg clear_request_pending_135 = 1'b0;
    reg clear_mem_seen_135 = 1'b0;
    reg [HIST_ADDR_WIDTH-1:0] clear_addr_135 = 0;
    reg clear_done_toggle_135 = 1'b0;
    reg accumulation_overflow_135 = 1'b0;

    reg [31:0] dwell_counter_135 = DEFAULT_DWELL_CYCLES;
    reg [HIST_ADDR_WIDTH-1:0] bin_index = 0;
    reg [CUM_WIDTH-1:0] last_boundary_snapshot_135 = 0;
    reg [31:0] pass_count_binary_135 = 0;
    reg [31:0] pass_snapshot_data_135 = 0;
    reg pass_snapshot_toggle_135 = 0;
    reg pass_ack_meta_135 = 0, pass_ack_sync_135 = 0;

    // Live-histogram descriptor sidecar observes committed histogram writes only.
    reg [8:0] live_block_fill_135 = 0;
    reg [HIST_ADDR_WIDTH-1:0] live_block_start_135 = 0;
    reg live_pending_valid_135 = 0;
    reg [HIST_ADDR_WIDTH-1:0] live_pending_start_135 = 0;
    reg [HIST_ADDR_WIDTH:0] live_pending_count_135 = 0;
    reg [31:0] live_pending_pass_135 = 0;
    reg [HIST_ADDR_WIDTH-1:0] live_publish_start_135 = 0;
    reg [HIST_ADDR_WIDTH:0] live_publish_count_135 = 0;
    reg [31:0] live_publish_pass_135 = 0;
    reg live_publish_toggle_135 = 0;
    reg live_publish_ack_27 = 0;
    reg live_publish_ack_meta_135 = 0, live_publish_ack_sync_135 = 0;
    reg live_stream_overflow_135 = 0;

    // Cumulative simple-counter baselines. CLEAR/START changes logical baselines,
    // never the physical counters. Existing two-word counter UART frame is kept.
    reg [CUM_WIDTH-1:0] signal_counter_baseline_135 = 0;
    reg [CUM_WIDTH-1:0] trigger_counter_baseline_135 = 0;
    reg [31:0] signal_counter_snapshot_135 = 0;
    reg [31:0] trigger_counter_snapshot_135 = 0;
    reg [31:0] counter_sample_id_135 = 0;
    reg [31:0] counter_sample_snapshot_135 = 0;
    reg counter_snapshot_toggle_135 = 0;
    reg [31:0] counter_dwell_135 = DEFAULT_DWELL_CYCLES;
    reg counter_dwell_terminal_135 = (DEFAULT_DWELL_CYCLES <= 1);
    reg simple_counter_clear_seen_135 = 1'b0;
    wire [CUM_WIDTH-1:0] signal_counter_delta_135 =
        cumulative_event_counter_135 - signal_counter_baseline_135;
    wire [CUM_WIDTH-1:0] trigger_counter_delta_135 =
        cumulative_trigger_counter_135 - trigger_counter_baseline_135;

    // Independent simple-counter engine. This block has no dependency on the
    // MCS accumulator_state, bin_index, snapshot FIFO, or histogram RMW FSM.
    // Physical counters above run continuously. CLEAR/START only capture baselines.
    always @(posedge clk_150m) begin
        if (!acq_clock_ready) begin
            signal_counter_baseline_135 <= cumulative_event_counter_135;
            trigger_counter_baseline_135 <= cumulative_trigger_counter_135;
            signal_counter_snapshot_135 <= 32'd0;
            trigger_counter_snapshot_135 <= 32'd0;
            counter_sample_id_135 <= 32'd0;
            counter_sample_snapshot_135 <= 32'd0;
            counter_snapshot_toggle_135 <= 1'b0;
            counter_dwell_135 <= DEFAULT_DWELL_CYCLES;
            counter_dwell_terminal_135 <= (DEFAULT_DWELL_CYCLES <= 1);
            simple_counter_clear_seen_135 <= clear_sync_135;
        end else begin
            // CLEAR is logical only: zero the externally visible measurement by
            // moving the baselines to the continuously running physical counters.
            if (clear_sync_135 != simple_counter_clear_seen_135) begin
                simple_counter_clear_seen_135 <= clear_sync_135;
                signal_counter_baseline_135 <= cumulative_event_counter_135;
                trigger_counter_baseline_135 <= cumulative_trigger_counter_135;
                signal_counter_snapshot_135 <= 32'd0;
                trigger_counter_snapshot_135 <= 32'd0;
                counter_sample_id_135 <= 32'd0;
                counter_sample_snapshot_135 <= 32'd0;
                counter_dwell_135 <= dwell_reload_135;
                counter_dwell_terminal_135 <= (dwell_reload_135 <= 1);
            end else if (counter_command_sync_135 != counter_command_seen_135) begin
                // START counter mode: establish a fresh logical zero.
                signal_counter_baseline_135 <= cumulative_event_counter_135;
                trigger_counter_baseline_135 <= cumulative_trigger_counter_135;
                signal_counter_snapshot_135 <= 32'd0;
                trigger_counter_snapshot_135 <= 32'd0;
                counter_sample_id_135 <= 32'd0;
                counter_sample_snapshot_135 <= 32'd0;
                counter_dwell_135 <= dwell_sync_135;
                counter_dwell_terminal_135 <= (dwell_sync_135 <= 1);
            end else if (counter_mode_135 && run_enabled_135) begin
                if (!counter_dwell_terminal_135) begin
                    counter_dwell_135 <= counter_dwell_135 - 1'b1;
                    counter_dwell_terminal_135 <= (counter_dwell_135 == 2);
                end else begin
                    // The legacy UART frame reports the low 32 bits explicitly.
                    signal_counter_snapshot_135 <= signal_counter_delta_135[31:0];
                    trigger_counter_snapshot_135 <= trigger_counter_delta_135[31:0];
                    signal_counter_baseline_135 <= cumulative_event_counter_135;
                    trigger_counter_baseline_135 <= cumulative_trigger_counter_135;
                    counter_sample_id_135 <= counter_sample_id_135 + 1'b1;
                    counter_sample_snapshot_135 <= counter_sample_id_135 + 1'b1;
                    counter_snapshot_toggle_135 <= ~counter_snapshot_toggle_135;
                    counter_dwell_135 <= dwell_reload_135;
                    counter_dwell_terminal_135 <= (dwell_reload_135 <= 1);
                end
            end else begin
                counter_dwell_135 <= dwell_reload_135;
                counter_dwell_terminal_135 <= (dwell_reload_135 <= 1);
            end
        end
    end

    // Compatibility name: FIFO overflow flag in MCS6 now reports snapshot FIFO.
    wire bin_fifo_overflow_135 = snap_fifo_overflow_135;

    openmcs_histogram_bram #(.ADDR_WIDTH(HIST_ADDR_WIDTH), .DATA_WIDTH(32), .DEPTH(MAX_BINS))
    histogram_memory (
        .clock(clk_150m), .enable(ram_enable_135), .write_enable(ram_write_enable_135),
        .address(ram_address_135), .write_data(ram_write_data_135), .read_data(ram_read_data_135)
    );

    wire accumulator_start = histogram_ready_135 && !clear_active_135 &&
                             accumulator_state == ACC_IDLE && !snap_empty;

    always @* begin
        ram_enable_135 = 1'b0;
        ram_write_enable_135 = 1'b0;
        ram_address_135 = 0;
        ram_write_data_135 = 0;
        if (clear_active_135) begin
            ram_enable_135 = 1'b1; ram_write_enable_135 = 1'b1;
            ram_address_135 = clear_addr_135; ram_write_data_135 = 0;
        end else if (histogram_ready_135 && accumulator_state == ACC_WRITE) begin
            ram_enable_135 = 1'b1; ram_write_enable_135 = 1'b1;
            ram_address_135 = accumulator_addr;
            ram_write_data_135 = accumulator_sum[32] ? 32'hFFFFFFFF : accumulator_sum[31:0];
        end else if (accumulator_start) begin
            ram_enable_135 = 1'b1; ram_write_enable_135 = 1'b0;
            ram_address_135 = snap_fifo_addr[snap_rd_ptr];
        end else if (uart_read_pending_135) begin
            ram_enable_135 = 1'b1; ram_write_enable_135 = 1'b0;
            ram_address_135 = uart_read_addr_135;
        end
    end

    wire final_mcs_boundary_135 = acquiring && (dwell_counter_135 == 1) &&
        (bin_index == last_bin_135);

    // A trigger is accepted only while the MCS is idle and armed.  Trigger
    // edges received during a pass are deliberately ignored: they must have
    // no effect on bin timing and must never be deferred to pass completion.
    wire trigger_eligible_135 = !counter_mode_135 && run_enabled_135 &&
        mcs_trigger_edge && accept_sync_135;
    wire accept_idle_trigger_135 = trigger_eligible_135 && !acquiring && armed_135;
    wire accept_mcs_trigger_135 = accept_idle_trigger_135;

    // CDC/control and arming. Physical counters are deliberately absent here.
    always @(posedge clk_150m) begin
        pass_ack_meta_135 <= pass_ack_toggle_27;
        pass_ack_sync_135 <= pass_ack_meta_135;
        // Either a successful CFG5 or explicit X command requests exactly one
        // clear. XOR composes the two independent event toggles safely.
        clear_meta_135 <= clear_toggle_27 ^ explicit_clear_toggle_27;
        clear_sync_135 <= clear_meta_135;
        run_command_meta_135 <= run_command_toggle_27; run_command_sync_135 <= run_command_meta_135;
        counter_command_meta_135 <= counter_command_toggle_27; counter_command_sync_135 <= counter_command_meta_135;
        stop_command_meta_135 <= stop_command_toggle_27; stop_command_sync_135 <= stop_command_meta_135;
        uart_req_meta_135 <= uart_ram_request_toggle_27; uart_req_sync_135 <= uart_req_meta_135; uart_req_safe_135 <= uart_req_sync_135;
        uart_addr_meta_135 <= uart_ram_request_addr_27; uart_addr_sync_135 <= uart_addr_meta_135;
        bins_meta_135 <= armed_bins_27; bins_sync_135 <= bins_meta_135;
        dwell_meta_135 <= armed_dwell_27; dwell_sync_135 <= dwell_meta_135;
        live_enable_meta_135 <= live_enable_27; live_enable_sync_135 <= live_enable_meta_135;
        pass_limit_meta_135 <= pass_limit_27; pass_limit_sync_135 <= pass_limit_meta_135;
        trigger_falling_meta_135 <= trigger_falling_27; trigger_falling_sync_135 <= trigger_falling_meta_135;

        if (!acq_clock_ready) begin
            mcs_state_135 <= MCS_STOPPED; run_enabled_135 <= 0; counter_mode_135 <= 0;
            passes_remaining_135 <= 0; quota_limited_135 <= 0;
            clear_arm_seen_135 <= clear_sync_135;
            run_command_seen_135 <= run_command_sync_135;
            counter_command_seen_135 <= counter_command_sync_135;
            stop_command_seen_135 <= stop_command_sync_135;
        end else if (clear_sync_135 != clear_arm_seen_135) begin
            clear_arm_seen_135 <= clear_sync_135; run_enabled_135 <= 0; counter_mode_135 <= 0;
            if (!acquiring) mcs_state_135 <= MCS_STOPPED;
        end else if (!histogram_ready_135) begin
            run_enabled_135 <= 0; counter_mode_135 <= 0; mcs_state_135 <= MCS_STOPPED;
        end else if (stop_command_sync_135 != stop_command_seen_135) begin
            stop_command_seen_135 <= stop_command_sync_135; run_enabled_135 <= 0; counter_mode_135 <= 0;
            if (!acquiring) mcs_state_135 <= MCS_STOPPED;
        end else if (run_command_sync_135 != run_command_seen_135) begin
            run_command_seen_135 <= run_command_sync_135; counter_mode_135 <= 0;
            dwell_reload_135 <= dwell_sync_135; last_bin_135 <= bins_sync_135[HIST_ADDR_WIDTH-1:0] - 1'b1;
            if (pass_limit_sync_135 == 0) begin
                run_enabled_135 <= 1'b1;
                passes_remaining_135 <= 32'd0;
                quota_limited_135 <= 1'b0;
                if (!acquiring) mcs_state_135 <= MCS_ARMED;
            end else if (pass_count_binary_135 < pass_limit_sync_135) begin
                run_enabled_135 <= 1'b1;
                passes_remaining_135 <= pass_limit_sync_135 - pass_count_binary_135;
                quota_limited_135 <= 1'b1;
                if (!acquiring) mcs_state_135 <= MCS_ARMED;
            end else begin
                run_enabled_135 <= 1'b0;
                passes_remaining_135 <= 32'd0;
                quota_limited_135 <= 1'b1;
                if (!acquiring) mcs_state_135 <= MCS_STOPPED;
            end
        end else if (counter_command_sync_135 != counter_command_seen_135) begin
            counter_command_seen_135 <= counter_command_sync_135; run_enabled_135 <= 1; counter_mode_135 <= 1;
            dwell_reload_135 <= dwell_sync_135;
            if (!acquiring) mcs_state_135 <= MCS_ARMED;
        end else if (accept_mcs_trigger_135) begin
            // Consume the arm only when an idle-state trigger is actually accepted.
            mcs_state_135 <= MCS_ACTIVE;
            if (quota_limited_135) begin
                passes_remaining_135 <= passes_remaining_135 - 1'b1;
                // Disarm future launches on the same acquisition-clock edge
                // that accepts the final allowed trigger. The active pass is
                // not cancelled and therefore drains through its final bin.
                if (passes_remaining_135 == 1)
                    run_enabled_135 <= 1'b0;
            end
        end else if (final_mcs_boundary_135) begin
            mcs_state_135 <= run_enabled_135 ? MCS_ARMED : MCS_STOPPED;
        end
    end

    // Compatibility status registers remain present for the existing MCS6
    // frame format, but queued/deferred triggering is disabled.  These values
    // therefore remain zero for every run.
    always @(posedge clk_150m) begin
        pending_trigger_count_135 <= 32'd0;
        trigger_queue_overflow_135 <= 1'b0;
    end

    assign led1_n = ~armed_135;
    assign pass_toggle_out = accepted_pass_toggle_135;
    assign stop_ack_out = ~run_enabled_135;

    // A full histogram is immutable only after acquisition has stopped and all
    // queued bin snapshots have committed to BRAM. Live block frames remain
    // previews; this level authorizes the one coherent final full-frame read.
    wire final_histogram_ready_135 = histogram_ready_135 &&
        !run_enabled_135 && !acquiring && !clear_active_135 &&
        !clear_request_pending_135 && snap_empty &&
        (accumulator_state == ACC_IDLE) &&
        (pass_snapshot_data_135 == pass_count_binary_135);

    // Acquisition + background processing. Boundary convention is [ti,ti+1).
    // Fine cumulative prefixes place every boundary at the accepted trigger's
    // fine (3.333 ns) phase. An event coincident with a boundary is excluded from
    // the closing snapshot and therefore belongs exactly once to the new bin.
    always @(posedge clk_150m) begin
        live_publish_ack_meta_135 <= live_publish_ack_27;
        live_publish_ack_sync_135 <= live_publish_ack_meta_135;

        if (!acq_clock_ready) begin
            bin_index <= 0; dwell_counter_135 <= DEFAULT_DWELL_CYCLES;
            last_boundary_snapshot_135 <= 0;
            active_trigger_phase_135 <= 2'd0;
            snap_wr_ptr <= 0; snap_rd_ptr <= 0; snap_count <= 0; snap_fifo_overflow_135 <= 0;
            accumulator_state <= ACC_IDLE; histogram_ready_135 <= 0; clear_active_135 <= 0;
            clear_request_pending_135 <= 0; clear_mem_seen_135 <= clear_sync_135; clear_addr_135 <= 0;
            accumulation_overflow_135 <= 0; pass_count_binary_135 <= 0;
            pass_snapshot_data_135 <= 0; pass_snapshot_toggle_135 <= pass_ack_sync_135;
            uart_req_seen_135 <= uart_req_safe_135; uart_read_pending_135 <= 0; ram_read_tag_135 <= RAM_READ_NONE;
            live_block_fill_135 <= 0; live_pending_valid_135 <= 0; live_publish_toggle_135 <= live_publish_ack_sync_135;
            live_stream_overflow_135 <= 0;
        end else begin
            // Absolute pass-count mailbox.
            if (pass_snapshot_toggle_135 == pass_ack_sync_135 && pass_snapshot_data_135 != pass_count_binary_135) begin
                pass_snapshot_data_135 <= pass_count_binary_135;
                pass_snapshot_toggle_135 <= ~pass_snapshot_toggle_135;
            end
            if (clear_sync_135 != clear_mem_seen_135) begin
                clear_mem_seen_135 <= clear_sync_135; clear_request_pending_135 <= 1;
                // CLEAR for simple counters is baseline-only; physical counters continue.
            end
            if (uart_req_safe_135 != uart_req_seen_135) begin
                uart_req_seen_135 <= uart_req_safe_135; uart_read_addr_135 <= uart_addr_sync_135; uart_read_pending_135 <= 1;
            end

            if (clear_active_135) begin
                pass_count_binary_135 <= 0; accumulator_state <= ACC_IDLE; snap_wr_ptr <= 0; snap_rd_ptr <= 0; snap_count <= 0;
                if (clear_addr_135 == MAX_BINS-1) begin
                    clear_active_135 <= 0; clear_request_pending_135 <= 0; histogram_ready_135 <= 1; clear_addr_135 <= 0;
                    clear_done_toggle_135 <= ~clear_done_toggle_135;
                end else clear_addr_135 <= clear_addr_135 + 1'b1;
            end else if (!histogram_ready_135) begin
                clear_active_135 <= 1; clear_addr_135 <= 0; accumulator_state <= ACC_IDLE;
                snap_wr_ptr <= 0; snap_rd_ptr <= 0; snap_count <= 0; accumulation_overflow_135 <= 0; snap_fifo_overflow_135 <= 0;
                pass_count_binary_135 <= 0;
            end else if (clear_request_pending_135 && !acquiring && snap_count==0 && accumulator_state==ACC_IDLE) begin
                histogram_ready_135 <= 0; clear_active_135 <= 1; clear_addr_135 <= 0;
                accumulation_overflow_135 <= 0; snap_fifo_overflow_135 <= 0;
            end else begin
                // Complete prior UART BRAM read.
                if (ram_read_tag_135 == RAM_READ_UART) begin
                    uart_response_data_135 <= ram_read_data_135;
                    uart_response_toggle_135 <= ~uart_response_toggle_135;
                end
                ram_read_tag_135 <= RAM_READ_NONE;

                // Background cumulative differencer + histogram RMW.
                if (accumulator_start) begin
                    accumulator_addr <= snap_fifo_addr[snap_rd_ptr];
                    accumulator_curr <= snap_fifo_curr[snap_rd_ptr];
                    accumulator_prev <= snap_fifo_prev[snap_rd_ptr];
                    accumulator_last <= snap_fifo_last[snap_rd_ptr];
                    snap_rd_ptr <= snap_rd_ptr + 1'b1;
                    accumulator_state <= ACC_CAPTURE;
                end else if (accumulator_state == ACC_CAPTURE) begin
                    accumulator_sum <= {1'b0,ram_read_data_135} +
                        {1'b0,accumulator_delta_mod[31:0]};
                    accumulator_state <= ACC_WRITE;
                end else if (accumulator_state == ACC_WRITE) begin
                    if (accumulator_sum[32]) accumulation_overflow_135 <= 1;
                    if (accumulator_last) pass_count_binary_135 <= pass_count_binary_135 + 1'b1;
                    accumulator_state <= ACC_IDLE;

                    // Live descriptor is based only on committed histogram bins.
                    if (live_enable_sync_135) begin
                        if (live_block_fill_135 == 0) live_block_start_135 <= accumulator_addr;
                        if (accumulator_last || live_block_fill_135 == LIVE_BLOCK_BINS-1) begin
                            if (live_pending_valid_135 && live_publish_toggle_135 != live_publish_ack_sync_135)
                                live_stream_overflow_135 <= 1;
                            live_pending_valid_135 <= 1;
                            live_pending_start_135 <= live_block_start_135;
                            live_pending_count_135 <= live_block_fill_135 + 1'b1;
                            live_pending_pass_135 <= pass_count_binary_135 + (accumulator_last ? 1'b1 : 1'b0);
                            live_block_fill_135 <= 0;
                        end else live_block_fill_135 <= live_block_fill_135 + 1'b1;
                    end
                end

                if (live_pending_valid_135 && live_publish_toggle_135 == live_publish_ack_sync_135) begin
                    live_publish_start_135 <= live_pending_start_135;
                    live_publish_count_135 <= live_pending_count_135;
                    live_publish_pass_135 <= live_pending_pass_135;
                    live_publish_toggle_135 <= ~live_publish_toggle_135;
                    live_pending_valid_135 <= 0;
                end

                // UART histogram read gets an idle BRAM cycle only.
                if (uart_read_pending_135 && accumulator_state==ACC_IDLE && !accumulator_start) begin
                    ram_read_tag_135 <= RAM_READ_UART; uart_read_pending_135 <= 0;
                end

                // Snapshot FIFO occupancy: processor pop and acquisition push can coincide.
                case ({(acquiring && dwell_counter_135==1 && (!snap_full || accumulator_start)), accumulator_start})
                    2'b10: snap_count <= snap_count + 1'b1;
                    2'b01: snap_count <= snap_count - 1'b1;
                    default: begin end
                endcase

                // MCS launch. Baseline is the cumulative prefix immediately
                // before the accepted trigger phase, so a simultaneous event
                // is included in bin 0 exactly once.
                if (accept_idle_trigger_135) begin
                    bin_index <= 0; dwell_counter_135 <= dwell_reload_135;
                    active_trigger_phase_135 <= mcs_trigger_phase;
                    last_boundary_snapshot_135 <= launch_phase_event_prefix;
                    accepted_pass_toggle_135 <= ~accepted_pass_toggle_135;
                end else if (acquiring) begin
                    if (dwell_counter_135 > 1) begin
                        dwell_counter_135 <= dwell_counter_135 - 1'b1;
                    end else begin
                        // Close current half-open bin with pre-increment snapshot.
                        if (!snap_full || accumulator_start) begin
                            snap_fifo_addr[snap_wr_ptr] <= bin_index;
                            snap_fifo_curr[snap_wr_ptr] <= active_phase_event_prefix;
                            snap_fifo_prev[snap_wr_ptr] <= last_boundary_snapshot_135;
                            snap_fifo_last[snap_wr_ptr] <= (bin_index == last_bin_135);
                            snap_wr_ptr <= snap_wr_ptr + 1'b1;
                        end else snap_fifo_overflow_135 <= 1;
                        last_boundary_snapshot_135 <= active_phase_event_prefix;
                        dwell_counter_135 <= dwell_reload_135;
                        if (bin_index == last_bin_135) begin
                            // Finish this pass and wait for a new physical H2
                            // edge. Any H2 received while acquiring was ignored.
                        end else bin_index <= bin_index + 1'b1;
                    end
                end

            end
        end
    end

    // ----------------------- 50 MHz control/readout domain -------------
    // UART words cross through a bundled-data toggle handshake. The RAM itself
    // is never accessed from this clock domain.
    reg pll_lock_meta_27 = 1'b0, pll_lock_sync_27 = 1'b0;
    reg pass_snapshot_meta_27 = 1'b0;
    reg pass_snapshot_sync_27 = 1'b0;
    reg pass_snapshot_safe_27 = 1'b0;
    reg pass_snapshot_seen_27 = 1'b0;
    reg [31:0] pass_data_meta_27 = 32'd0;
    reg [31:0] pass_data_sync_27 = 32'd0;
    reg ready_meta_27 = 1'b0, ready_sync_27 = 1'b0;
    reg clear_done_meta_27 = 1'b0, clear_done_sync_27 = 1'b0;
    reg overflow_meta_27 = 1'b0, overflow_sync_27 = 1'b0;
    reg fifo_overflow_meta_27 = 1'b0, fifo_overflow_sync_27 = 1'b0;
    reg trigger_queue_overflow_meta_27 = 1'b0, trigger_queue_overflow_sync_27 = 1'b0;
    reg uart_rsp_meta_27 = 1'b0, uart_rsp_sync_27 = 1'b0;
    reg uart_rsp_safe_27 = 1'b0, uart_rsp_seen_27 = 1'b0;
    reg [31:0] uart_data_meta_27 = 32'd0;
    reg [31:0] uart_data_sync_27 = 32'd0;
    // Bundled-data Counter snapshot crossing. The 100 MHz source holds both
    // words stable until the next Counter dwell boundary.
    reg counter_snapshot_meta_27 = 1'b0;
    reg counter_snapshot_sync_27 = 1'b0;
    reg counter_snapshot_safe_27 = 1'b0;
    reg counter_snapshot_seen_27 = 1'b0;
    reg [31:0] signal_counter_meta_27 = 32'd0;
    reg [31:0] signal_counter_sync_27 = 32'd0;
    reg [31:0] trigger_counter_meta_27 = 32'd0;
    reg [31:0] trigger_counter_sync_27 = 32'd0;
    reg [31:0] counter_sample_meta_27 = 32'd0;
    reg [31:0] counter_sample_sync_27 = 32'd0;
    reg live_publish_meta_27 = 1'b0;
    reg live_publish_sync_27 = 1'b0;
    reg live_publish_safe_27 = 1'b0;
    reg live_publish_seen_27 = 1'b0;
    reg [HIST_ADDR_WIDTH-1:0] live_start_meta_27 = {HIST_ADDR_WIDTH{1'b0}};
    reg [HIST_ADDR_WIDTH-1:0] live_start_sync_27 = {HIST_ADDR_WIDTH{1'b0}};
    reg [HIST_ADDR_WIDTH:0] live_count_meta_27 = {(HIST_ADDR_WIDTH+1){1'b0}};
    reg [HIST_ADDR_WIDTH:0] live_count_sync_27 = {(HIST_ADDR_WIDTH+1){1'b0}};
    reg [31:0] live_pass_meta_27 = 32'd0;
    reg [31:0] live_pass_sync_27 = 32'd0;
    reg live_overflow_meta_27 = 1'b0;
    reg live_overflow_sync_27 = 1'b0;
    reg final_ready_meta_27 = 1'b0;
    reg final_ready_sync_27 = 1'b0;
    reg final_ready_safe_27 = 1'b0;
    always @(posedge clk_50m) begin
        pll_lock_meta_27 <= pll_lock;
        pll_lock_sync_27 <= pll_lock_meta_27;
        pass_snapshot_meta_27 <= pass_snapshot_toggle_135;
        pass_snapshot_sync_27 <= pass_snapshot_meta_27;
        pass_snapshot_safe_27 <= pass_snapshot_sync_27;
        pass_data_meta_27 <= pass_snapshot_data_135;
        pass_data_sync_27 <= pass_data_meta_27;
        ready_meta_27 <= histogram_ready_135;
        ready_sync_27 <= ready_meta_27;
        clear_done_meta_27 <= clear_done_toggle_135;
        clear_done_sync_27 <= clear_done_meta_27;
        overflow_meta_27 <= accumulation_overflow_135;
        overflow_sync_27 <= overflow_meta_27;
        fifo_overflow_meta_27 <= bin_fifo_overflow_135;
        fifo_overflow_sync_27 <= fifo_overflow_meta_27;
        trigger_queue_overflow_meta_27 <= trigger_queue_overflow_135;
        trigger_queue_overflow_sync_27 <= trigger_queue_overflow_meta_27;
        uart_rsp_meta_27 <= uart_response_toggle_135;
        uart_rsp_sync_27 <= uart_rsp_meta_27;
        uart_rsp_safe_27 <= uart_rsp_sync_27;
        uart_data_meta_27 <= uart_response_data_135;
        uart_data_sync_27 <= uart_data_meta_27;
        counter_snapshot_meta_27 <= counter_snapshot_toggle_135;
        counter_snapshot_sync_27 <= counter_snapshot_meta_27;
        counter_snapshot_safe_27 <= counter_snapshot_sync_27;
        signal_counter_meta_27 <= signal_counter_snapshot_135;
        signal_counter_sync_27 <= signal_counter_meta_27;
        trigger_counter_meta_27 <= trigger_counter_snapshot_135;
        trigger_counter_sync_27 <= trigger_counter_meta_27;
        counter_sample_meta_27 <= counter_sample_snapshot_135;
        counter_sample_sync_27 <= counter_sample_meta_27;
        live_publish_meta_27 <= live_publish_toggle_135;
        live_publish_sync_27 <= live_publish_meta_27;
        live_publish_safe_27 <= live_publish_sync_27;
        live_start_meta_27 <= live_publish_start_135;
        live_start_sync_27 <= live_start_meta_27;
        live_count_meta_27 <= live_publish_count_135;
        live_count_sync_27 <= live_count_meta_27;
        live_pass_meta_27 <= live_publish_pass_135;
        live_pass_sync_27 <= live_pass_meta_27;
        live_overflow_meta_27 <= live_stream_overflow_135;
        live_overflow_sync_27 <= live_overflow_meta_27;
        final_ready_meta_27 <= final_histogram_ready_135;
        final_ready_sync_27 <= final_ready_meta_27;
        final_ready_safe_27 <= final_ready_sync_27;
    end

    reg [15:0] configured_bins_27 = DEFAULT_BINS;
    reg [31:0] configured_dwell_27 = DEFAULT_DWELL_CYCLES;
    reg [15:0] active_bins_27 = DEFAULT_BINS;
    reg [31:0] active_dwell_27 = DEFAULT_DWELL_CYCLES;

    reg [3:0] state = ST_IDLE;
    reg cfg_request_seen_27 = 1'b0;
    reg clear_done_seen_27 = 1'b0;
    reg cfg_ack_requires_clear = 1'b0;
    reg [31:0] accumulated_passes_27 = 32'd0;
    reg [31:0] last_frame_pass_count_27 = 32'd0;
    reg [31:0] frame_pass_count_27 = 32'd0;
    reg frame_overflow_27 = 1'b0;
    reg frame_fifo_overflow_27 = 1'b0;
    reg frame_trigger_queue_overflow_27 = 1'b0;
    reg frame_is_counter_27 = 1'b0;
    reg frame_is_live_27 = 1'b0;
    reg frame_is_atomic_final_27 = 1'b0;
    reg final_ready_consumed_27 = 1'b0;
    reg [HIST_ADDR_WIDTH-1:0] frame_live_start_27 = {HIST_ADDR_WIDTH{1'b0}};
    reg [HIST_ADDR_WIDTH:0] frame_live_count_27 = {(HIST_ADDR_WIDTH+1){1'b0}};
    reg [31:0] frame_signal_count_27 = 32'd0;
    reg [31:0] frame_trigger_count_27 = 32'd0;
    reg live_frame_pending_27 = 1'b0;
    reg [HIST_ADDR_WIDTH-1:0] pending_live_start_27 = {HIST_ADDR_WIDTH{1'b0}};
    reg [HIST_ADDR_WIDTH:0] pending_live_count_27 = {(HIST_ADDR_WIDTH+1){1'b0}};
    reg [31:0] pending_live_pass_27 = 32'd0;
    reg pending_live_overflow_27 = 1'b0;

    reg [4:0] header_index = 5'd0;
    reg [HIST_ADDR_WIDTH-1:0] word_index = {HIST_ADDR_WIDTH{1'b0}};
    reg [1:0] data_byte_index = 2'd0;
    reg [2:0] total_byte_index = 3'd0;
    reg [1:0] trailer_index = 2'd0;
    reg [31:0] data_word = 32'd0;
    reg [63:0] histogram_total = 64'd0;
    reg [3:0] ack_index = 4'd0;
    reg [7:0] ack_status = 8'd0;
    reg [15:0] ack_bins = DEFAULT_BINS;
    reg [31:0] ack_dwell = DEFAULT_DWELL_CYCLES;
    reg [7:0] ack_crc = 8'h00;

    // Full MCS6 header remains compatible; previously reserved flag bit 4 marks
    // a quiescent atomic final frame. A full frame without bit 4 is a
    // non-atomic live preview.
    // Live MCB1 header (23 bytes total) carries total bins, start bin, block
    // length, dwell, clock, in-progress pass number, and flags.  MCB1 bit 3
    // identifies a live block; bit 2 reports a skipped descriptor.
    function [7:0] header_byte;
        input [4:0] index;
        begin
            if (frame_is_live_27) begin
                case (index)
                    5'd0: header_byte = "M";
                    5'd1: header_byte = "C";
                    5'd2: header_byte = "B";
                    5'd3: header_byte = "1";
                    5'd4: header_byte = active_bins_27[7:0];
                    5'd5: header_byte = active_bins_27[15:8];
                    5'd6: header_byte = frame_live_start_27[7:0];
                    5'd7: header_byte = frame_live_start_27[HIST_ADDR_WIDTH-1:8];
                    5'd8: header_byte = frame_live_count_27[7:0];
                    5'd9: header_byte = frame_live_count_27[HIST_ADDR_WIDTH:8];
                    5'd10: header_byte = active_dwell_27[7:0];
                    5'd11: header_byte = active_dwell_27[15:8];
                    5'd12: header_byte = active_dwell_27[23:16];
                    5'd13: header_byte = active_dwell_27[31:24];
                    5'd14: header_byte = ACQ_CLOCK_HZ_U32[7:0];
                    5'd15: header_byte = ACQ_CLOCK_HZ_U32[15:8];
                    5'd16: header_byte = ACQ_CLOCK_HZ_U32[23:16];
                    5'd17: header_byte = ACQ_CLOCK_HZ_U32[31:24];
                    5'd18: header_byte = frame_pass_count_27[7:0];
                    5'd19: header_byte = frame_pass_count_27[15:8];
                    5'd20: header_byte = frame_pass_count_27[23:16];
                    5'd21: header_byte = frame_pass_count_27[31:24];
                    default: header_byte = {4'd0, 1'b1,
                        pending_live_overflow_27, frame_fifo_overflow_27,
                        frame_overflow_27};
                endcase
            end else begin
                case (index)
                    5'd0: header_byte = "M";
                    5'd1: header_byte = "C";
                    5'd2: header_byte = "S";
                    5'd3: header_byte = "6";
                    5'd4: header_byte = frame_is_counter_27 ? 8'd2 : active_bins_27[7:0];
                    5'd5: header_byte = frame_is_counter_27 ? 8'd0 : active_bins_27[15:8];
                    5'd6: header_byte = active_dwell_27[7:0];
                    5'd7: header_byte = active_dwell_27[15:8];
                    5'd8: header_byte = active_dwell_27[23:16];
                    5'd9: header_byte = active_dwell_27[31:24];
                    5'd10: header_byte = ACQ_CLOCK_HZ_U32[7:0];
                    5'd11: header_byte = ACQ_CLOCK_HZ_U32[15:8];
                    5'd12: header_byte = ACQ_CLOCK_HZ_U32[23:16];
                    5'd13: header_byte = ACQ_CLOCK_HZ_U32[31:24];
                    5'd14: header_byte = frame_pass_count_27[7:0];
                    5'd15: header_byte = frame_pass_count_27[15:8];
                    5'd16: header_byte = frame_pass_count_27[23:16];
                    5'd17: header_byte = frame_pass_count_27[31:24];
                    default: header_byte = {3'd0, frame_is_atomic_final_27,
                        frame_trigger_queue_overflow_27,
                        frame_is_counter_27, frame_fifo_overflow_27, frame_overflow_27};
                endcase
            end
        end
    endfunction

    function [7:0] word_byte;
        input [31:0] word_value;
        input [1:0] index;
        begin
            case (index)
                2'd0: word_byte = word_value[7:0];
                2'd1: word_byte = word_value[15:8];
                2'd2: word_byte = word_value[23:16];
                default: word_byte = word_value[31:24];
            endcase
        end
    endfunction

    function [7:0] total_byte;
        input [63:0] value;
        input [2:0] index;
        begin
            case (index)
                3'd0: total_byte = value[7:0];
                3'd1: total_byte = value[15:8];
                3'd2: total_byte = value[23:16];
                3'd3: total_byte = value[31:24];
                3'd4: total_byte = value[39:32];
                3'd5: total_byte = value[47:40];
                3'd6: total_byte = value[55:48];
                default: total_byte = value[63:56];
            endcase
        end
    endfunction

    function [7:0] trailer_byte;
        input [1:0] index;
        begin
            case (index)
                2'd0: trailer_byte = "D";
                2'd1: trailer_byte = "O";
                2'd2: trailer_byte = "N";
                default: trailer_byte = "E";
            endcase
        end
    endfunction

    function [7:0] ack_byte;
        input [3:0] index;
        begin
            case (index)
                4'd0: ack_byte = "A";
                4'd1: ack_byte = "C";
                4'd2: ack_byte = "K";
                4'd3: ack_byte = "5";
                4'd4: ack_byte = ack_status;
                4'd5: ack_byte = ack_bins[7:0];
                4'd6: ack_byte = ack_bins[15:8];
                4'd7: ack_byte = ack_dwell[7:0];
                4'd8: ack_byte = ack_dwell[15:8];
                4'd9: ack_byte = ack_dwell[23:16];
                4'd10: ack_byte = ack_dwell[31:24];
                default: ack_byte = ack_crc;
            endcase
        end
    endfunction

    reg [7:0] tx_data;
    reg tx_valid;
    wire tx_ready;
    wire tx_fire = tx_valid & tx_ready;
    always @* begin
        tx_data = 8'h00;
        tx_valid = 1'b0;
        case (state)
            ST_CFG_ACK: begin
                tx_data = ack_byte(ack_index);
                tx_valid = 1'b1;
            end
            ST_TX_HEADER: begin
                tx_data = header_byte(header_index);
                tx_valid = 1'b1;
            end
            ST_TX_DATA: begin
                tx_data = word_byte(data_word, data_byte_index);
                tx_valid = 1'b1;
            end
            ST_TX_TOTAL: begin
                tx_data = total_byte(histogram_total, total_byte_index);
                tx_valid = 1'b1;
            end
            ST_TX_TRAILER: begin
                tx_data = trailer_byte(trailer_index);
                tx_valid = 1'b1;
            end
            default: begin
                tx_data = 8'h00;
                tx_valid = 1'b0;
            end
        endcase
    end

    uart_tx_8n1 #(.CLOCK_HZ(CTRL_CLOCK_HZ), .BAUD(UART_BAUD)) uart_transmitter (
        .clk(clk_50m), .data(tx_data), .valid(tx_valid),
        .ready(tx_ready), .tx(uart_tx)
    );

    always @(posedge clk_50m) begin
        cfg_stop_request_27 <= 1'b0;

        // A new run/clear drives final_ready low and rearms exactly one atomic
        // full-frame publication for the next quiescent stop boundary.
        if (!final_ready_safe_27)
            final_ready_consumed_27 <= 1'b0;

        if (pass_snapshot_safe_27 != pass_snapshot_seen_27) begin
            pass_snapshot_seen_27 <= pass_snapshot_safe_27;
            accumulated_passes_27 <= pass_data_sync_27;
            pass_ack_toggle_27 <= pass_snapshot_safe_27;
        end

        // Capture every committed-bin block descriptor promptly and
        // acknowledge its CDC mailbox independently of UART activity.  If a
        // newer descriptor arrives before the prior one is transmitted, keep
        // the newest one and latch an explicit stream-overrun indication.
        if (live_publish_safe_27 != live_publish_seen_27) begin
            live_publish_seen_27 <= live_publish_safe_27;
            live_publish_ack_27 <= live_publish_safe_27;
            if (live_frame_pending_27)
                pending_live_overflow_27 <= 1'b1;
            live_frame_pending_27 <= 1'b1;
            pending_live_start_27 <= live_start_sync_27;
            pending_live_count_27 <= live_count_sync_27;
            pending_live_pass_27 <= live_pass_sync_27;
            if (live_overflow_sync_27)
                pending_live_overflow_27 <= 1'b1;
        end

        // Retain a CFG5 request received during a long histogram frame and
        // accept it as soon as the UART readout becomes idle.
        if ((state == ST_IDLE || state == ST_ARMED) &&
            cfg_request_toggle != cfg_request_seen_27) begin
            cfg_request_seen_27 <= cfg_request_toggle;
            // Configuration is a stop-and-preserve transaction. Acquisition is
            // restarted only by an explicit subsequent R or K command.
            // Histogram RAM and accumulated pass count are preserved; only
            // an explicit X command is allowed to clear accumulated data.
            cfg_stop_request_27 <= 1'b1;
            ack_index <= 4'd0;
            cfg_ack_requires_clear <= 1'b0;

            if (!cfg_checksum_ok) begin
                ack_status <= 8'd3;
                ack_bins <= configured_bins_27;
                ack_dwell <= configured_dwell_27;
                ack_crc <= ack_crc_value(
                    8'd3, configured_bins_27, configured_dwell_27);
            end else if (cfg_candidate_bins == 0 ||
                         cfg_candidate_bins > MAX_BINS) begin
                ack_status <= 8'd1;
                ack_bins <= configured_bins_27;
                ack_dwell <= configured_dwell_27;
                ack_crc <= ack_crc_value(
                    8'd1, configured_bins_27, configured_dwell_27);
            end else if (cfg_candidate_dwell < 3) begin
                ack_status <= 8'd2;
                ack_bins <= configured_bins_27;
                ack_dwell <= configured_dwell_27;
                ack_crc <= ack_crc_value(
                    8'd2, configured_bins_27, configured_dwell_27);
            end else begin
                configured_bins_27 <= cfg_candidate_bins;
                configured_dwell_27 <= cfg_candidate_dwell;
                armed_bins_27 <= cfg_candidate_bins;
                armed_dwell_27 <= cfg_candidate_dwell;
                live_enable_27 <= (cfg_candidate_dwell >= LIVE_MIN_DWELL_CYCLES);
                ack_status <= 8'd0;
                ack_bins <= cfg_candidate_bins;
                ack_dwell <= cfg_candidate_dwell;
                ack_crc <= ack_crc_value(
                    8'd0, cfg_candidate_bins, cfg_candidate_dwell);
                // Reconfiguration MUST NOT clear accumulated data.
                // Preserve histogram BRAM and pass count so acquisition can
                // resume on top of the existing spectrum.  The dedicated X
                // command remains the only geometry-preserving clear path.
                cfg_ack_requires_clear <= 1'b0;
                active_bins_27 <= cfg_candidate_bins;
                active_dwell_27 <= cfg_candidate_dwell;
                live_frame_pending_27 <= 1'b0;
                pending_live_overflow_27 <= 1'b0;
            end
            state <= ST_CFG_ACK;
        end else begin
            case (state)
                ST_IDLE: begin
                    if (!ready_sync_27) begin
                        accumulated_passes_27 <= 32'd0;
                        last_frame_pass_count_27 <= 32'd0;
                    end
                    if (pll_lock_sync_27 && ready_sync_27) begin
                        armed_bins_27 <= configured_bins_27;
                        armed_dwell_27 <= configured_dwell_27;
                        live_enable_27 <= (configured_dwell_27 >= LIVE_MIN_DWELL_CYCLES);
                        active_bins_27 <= configured_bins_27;
                        active_dwell_27 <= configured_dwell_27;
                        state <= ST_ARMED;
                    end
                end

                ST_CFG_ACK: begin
                    if (tx_fire) begin
                        if (ack_index == 4'd11) begin
                            ack_index <= 4'd0;
                            if (cfg_ack_requires_clear)
                                state <= ST_CLEAR_WAIT;
                            else if (pll_lock_sync_27 && ready_sync_27)
                                state <= ST_ARMED;
                            else
                                state <= ST_IDLE;
                        end else begin
                            ack_index <= ack_index + 1'b1;
                        end
                    end
                end

                ST_CLEAR_WAIT: begin
                    if (!pll_lock_sync_27) begin
                        state <= ST_IDLE;
                    end else if (ready_sync_27 &&
                                 clear_done_sync_27 != clear_done_seen_27) begin
                        clear_done_seen_27 <= clear_done_sync_27;
                        accumulated_passes_27 <= 32'd0;
                        last_frame_pass_count_27 <= 32'd0;
                        live_frame_pending_27 <= 1'b0;
                        pending_live_overflow_27 <= 1'b0;
                        active_bins_27 <= configured_bins_27;
                        active_dwell_27 <= configured_dwell_27;
                        state <= ST_ARMED;
                    end
                end

                ST_ARMED: begin
                    if (!pll_lock_sync_27 || !ready_sync_27) begin
                        state <= ST_IDLE;
                    end else if (counter_snapshot_safe_27 !=
                                 counter_snapshot_seen_27) begin
                        // Counter frames bypass histogram BRAM completely.
                        counter_snapshot_seen_27 <= counter_snapshot_safe_27;
                        frame_is_counter_27 <= 1'b1;
                        frame_signal_count_27 <= signal_counter_sync_27;
                        frame_trigger_count_27 <= trigger_counter_sync_27;
                        frame_pass_count_27 <= counter_sample_sync_27;
                        frame_overflow_27 <= 1'b0;
                        frame_fifo_overflow_27 <= 1'b0;
                        frame_trigger_queue_overflow_27 <= 1'b0;
                        frame_is_live_27 <= 1'b0;
                        frame_is_atomic_final_27 <= 1'b0;
                        header_index <= 5'd0;
                        histogram_total <= 64'd0;
                        state <= ST_TX_HEADER;
                    end else if (final_ready_safe_27 &&
                                 !final_ready_consumed_27 &&
                                 accumulated_passes_27 != 0) begin
                        frame_is_counter_27 <= 1'b0;
                        frame_pass_count_27 <= accumulated_passes_27;
                        frame_overflow_27 <= overflow_sync_27;
                        frame_fifo_overflow_27 <= fifo_overflow_sync_27;
                        frame_trigger_queue_overflow_27 <= trigger_queue_overflow_sync_27;
                        frame_is_live_27 <= 1'b0;
                        frame_is_atomic_final_27 <= 1'b1;
                        final_ready_consumed_27 <= 1'b1;
                        header_index <= 5'd0;
                        histogram_total <= 64'd0;
                        state <= ST_TX_HEADER;
                    end else if (!final_ready_safe_27 &&
                                 accumulated_passes_27 !=
                                 last_frame_pass_count_27) begin
                        // Full-size non-atomic preview. BRAM may continue to
                        // accumulate while this frame is read.
                        frame_is_counter_27 <= 1'b0;
                        frame_pass_count_27 <= accumulated_passes_27;
                        frame_overflow_27 <= overflow_sync_27;
                        frame_fifo_overflow_27 <= fifo_overflow_sync_27;
                        frame_trigger_queue_overflow_27 <= trigger_queue_overflow_sync_27;
                        frame_is_live_27 <= 1'b0;
                        frame_is_atomic_final_27 <= 1'b0;
                        header_index <= 5'd0;
                        histogram_total <= 64'd0;
                        state <= ST_TX_HEADER;
                    end else if (live_frame_pending_27 &&
                                 pending_live_pass_27 > last_frame_pass_count_27) begin
                        frame_is_counter_27 <= 1'b0;
                        frame_is_live_27 <= 1'b1;
                        frame_is_atomic_final_27 <= 1'b0;
                        frame_live_start_27 <= pending_live_start_27;
                        frame_live_count_27 <= pending_live_count_27;
                        frame_pass_count_27 <= pending_live_pass_27;
                        frame_overflow_27 <= overflow_sync_27;
                        frame_fifo_overflow_27 <= fifo_overflow_sync_27;
                        frame_trigger_queue_overflow_27 <= 1'b0;
                        live_frame_pending_27 <= 1'b0;
                        header_index <= 5'd0;
                        histogram_total <= 64'd0;
                        state <= ST_TX_HEADER;
                    end else if (live_frame_pending_27) begin
                        // The coherent final MCS6 frame already covers this
                        // older preview block, so the block is redundant.
                        live_frame_pending_27 <= 1'b0;
                    end
                end

                ST_TX_HEADER: begin
                    if (tx_fire) begin
                        if (header_index == (frame_is_live_27 ? 5'd22 : 5'd18)) begin
                            header_index <= 5'd0;
                            word_index <= {HIST_ADDR_WIDTH{1'b0}};
                            if (frame_is_counter_27) begin
                                data_word <= frame_signal_count_27;
                                histogram_total <= {32'd0, frame_signal_count_27};
                                data_byte_index <= 2'd0;
                                state <= ST_TX_DATA;
                            end else if (frame_is_live_27) begin
                                uart_ram_request_addr_27 <= frame_live_start_27;
                                uart_ram_request_toggle_27 <=
                                    ~uart_ram_request_toggle_27;
                                state <= ST_HIST_WAIT;
                            end else begin
                                uart_ram_request_addr_27 <=
                                    {HIST_ADDR_WIDTH{1'b0}};
                                uart_ram_request_toggle_27 <=
                                    ~uart_ram_request_toggle_27;
                                state <= ST_HIST_WAIT;
                            end
                        end else begin
                            header_index <= header_index + 1'b1;
                        end
                    end
                end

                ST_HIST_WAIT: begin
                    if (uart_rsp_safe_27 != uart_rsp_seen_27) begin
                        uart_rsp_seen_27 <= uart_rsp_safe_27;
                        data_word <= uart_data_sync_27;
                        histogram_total <=
                            histogram_total + uart_data_sync_27;
                        data_byte_index <= 2'd0;
                        state <= ST_TX_DATA;
                    end
                end

                ST_TX_DATA: begin
                    if (tx_fire) begin
                        if (data_byte_index == 2'd3) begin
                            data_byte_index <= 2'd0;
                            if (frame_is_counter_27) begin
                                if (word_index == 0) begin
                                    word_index <= {{(HIST_ADDR_WIDTH-1){1'b0}}, 1'b1};
                                    data_word <= frame_trigger_count_27;
                                    histogram_total <= histogram_total +
                                                       frame_trigger_count_27;
                                end else begin
                                    total_byte_index <= 3'd0;
                                    state <= ST_TX_TOTAL;
                                end
                            end else if (frame_is_live_27 &&
                                         word_index == frame_live_count_27 - 1'b1) begin
                                total_byte_index <= 3'd0;
                                state <= ST_TX_TOTAL;
                            end else if (!frame_is_live_27 &&
                                         word_index == active_bins_27 - 1'b1) begin
                                total_byte_index <= 3'd0;
                                state <= ST_TX_TOTAL;
                            end else begin
                                word_index <= word_index + 1'b1;
                                uart_ram_request_addr_27 <=
                                    (frame_is_live_27 ? frame_live_start_27 :
                                     {HIST_ADDR_WIDTH{1'b0}}) + word_index + 1'b1;
                                uart_ram_request_toggle_27 <=
                                    ~uart_ram_request_toggle_27;
                                state <= ST_HIST_WAIT;
                            end
                        end else begin
                            data_byte_index <= data_byte_index + 1'b1;
                        end
                    end
                end

                ST_TX_TOTAL: begin
                    if (tx_fire) begin
                        if (total_byte_index == 3'd7) begin
                            total_byte_index <= 3'd0;
                            trailer_index <= 2'd0;
                            state <= ST_TX_TRAILER;
                        end else begin
                            total_byte_index <= total_byte_index + 1'b1;
                        end
                    end
                end

                ST_TX_TRAILER: begin
                    if (tx_fire) begin
                        if (trailer_index == 2'd3) begin
                            trailer_index <= 2'd0;
                            if (!frame_is_live_27)
                                last_frame_pass_count_27 <= frame_pass_count_27;
                            frame_is_counter_27 <= 1'b0;
                            frame_is_live_27 <= 1'b0;
                            frame_is_atomic_final_27 <= 1'b0;
                            state <= ST_ARMED;
                        end else begin
                            trailer_index <= trailer_index + 1'b1;
                        end
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule

// Canonical synchronous single-port RAM inference for 16,384 x 32 bits.
// This is a native power-of-two depth requiring approximately 32 of the
// GW5A-25's 56 BSRAM blocks.  There is no bank-selection mux in the read path.
module openmcs_histogram_bram #(
    parameter integer ADDR_WIDTH = 14,
    parameter integer DATA_WIDTH = 32,
    parameter integer DEPTH = 10_000
) (
    input wire clock,
    input wire enable,
    input wire write_enable,
    input wire [ADDR_WIDTH-1:0] address,
    input wire [DATA_WIDTH-1:0] write_data,
    output reg [DATA_WIDTH-1:0] read_data = {DATA_WIDTH{1'b0}}
);
    reg [DATA_WIDTH-1:0] memory [0:DEPTH-1]
        /* synthesis syn_ramstyle="block_ram" */;

    always @(posedge clock) begin
        if (enable) begin
            if (write_enable)
                memory[address] <= write_data;
            else
                read_data <= memory[address];
        end
    end
endmodule