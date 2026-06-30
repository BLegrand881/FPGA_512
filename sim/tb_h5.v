`timescale 1ns/1ps
// =============================================================================
// tb_h5.v  --  Hardware-capture-driven testbench for rx_process_mux + lane_framer
//
// Stimulus comes from a real Keysight logic-analyzer capture of the ADC board
// (sine-all-5.h5).  The Python script sim/extract_h5_stim.py converts the
// capture into the hex files consumed here.
//
// Build (from repo root):
//   conda run python3 sim/extract_h5_stim.py      # once: generates hex files
//   iverilog -g2012 -o sim_tb_h5 sim/tb_h5.v fpga/UWB_Serial_Handler.v
//   vvp sim_tb_h5
//
// Or with the Makefile:
//   make sim-h5
//
// What is checked
// ---------------
// 1. rx_process_mux decoded words
//    Every out_valid event on each of the 4 lanes is compared against the
//    golden expected word for that channel (even = out_chsel=0, odd =
//    out_chsel=1).  Expected values were computed in Python by simulating the
//    DUT's counter + shift-register logic on the same stimulus vectors.
//
// 2. lane_framer serial output
//    A 12-bit shift register per lane tracks the MSB-first serial stream.
//    When the accumulated 12-bit pattern matches the lane's SYNC word
//    (0xA35 / 0xB46 / 0xC57 / 0xD68), a detection is logged.  At least one
//    SYNC hit per lane is expected within the 8-frame stimulus window.
//
// Stimulus timing
// ---------------
// Stimuli are driven on the negedge so the DUT samples them on the following
// posedge.  This matches the source-synchronous relationship in hardware where
// the ADC board launches data on one clock edge and the FPGA samples it a
// half-period later.  The Python golden-value computation uses the same
// stim[k] to update state that the DUT uses at posedge k+1.
// =============================================================================

module tb_h5;

    // =========================================================================
    // Parameters  (must match the rx_process_mux instantiation in top.v)
    // =========================================================================
    localparam int N_CH         = 8;
    localparam int N_ADC_PER_GP = 4;
    localparam int ADC_BITS     = 12;
    localparam int ZERO_CYCLES  = 13;
    localparam int GROUP_CYCLES = 64;
    localparam int N_GROUPS_FR  = 16;   // DUT's N_GROUPS parameter
    localparam int N_FRAMES     = 8;    // ADC frames to replay
    localparam int N_GROUPS     = N_FRAMES * N_GROUPS_FR;    // 128 groups
    localparam int N_CYCLES     = N_GROUPS * GROUP_CYCLES;   // 8192 stim cycles
    localparam int EXP_PER_CH   = N_GROUPS * N_ADC_PER_GP;  // 512 words/channel

    // 32 MHz nominal (matches j33_clk32mhz target in top.v)
    localparam real CLK_PERIOD  = 31.25;  // ns

    // =========================================================================
    // Stimulus and expected-value storage
    // =========================================================================
    reg [9:0]  stim_mem [0:N_CYCLES-1];  // bit[9]=sync, bit[8]=next_amps, [7:0]=data

    reg [11:0] exp_ch0 [0:EXP_PER_CH-1];
    reg [11:0] exp_ch1 [0:EXP_PER_CH-1];
    reg [11:0] exp_ch2 [0:EXP_PER_CH-1];
    reg [11:0] exp_ch3 [0:EXP_PER_CH-1];
    reg [11:0] exp_ch4 [0:EXP_PER_CH-1];
    reg [11:0] exp_ch5 [0:EXP_PER_CH-1];
    reg [11:0] exp_ch6 [0:EXP_PER_CH-1];
    reg [11:0] exp_ch7 [0:EXP_PER_CH-1];

    // =========================================================================
    // DUT I/O
    // =========================================================================
    reg        clk      = 1'b0;
    reg        rst_n    = 1'b0;
    reg [7:0]  data_in  = 8'h0;
    reg        nxt_amps = 1'b0;
    reg        sync_in  = 1'b0;

    wire [3:0]  out_valid;
    wire [47:0] out_word;    // packed: lane p = out_word[p*12 +: 12]
    wire [3:0]  out_chsel;   // 0=even channel served, 1=odd
    wire [3:0]  serial_lane; // MSB-first serial stream per lane

    // =========================================================================
    // Clock
    // =========================================================================
    always #(CLK_PERIOD / 2.0) clk = ~clk;

    // =========================================================================
    // DUT: rx_process_mux  (parameters match top.v section 5)
    // =========================================================================
    rx_process_mux #(
        .N_CH        (8),
        .N_ADC_PER_GP(4),
        .ADC_BITS    (12),
        .ZERO_CYCLES (13),
        .GROUP_CYCLES(64),
        .N_GROUPS    (16),
        .SAMPS_PER_FR(64),
        .RX_FIFO_DEPTH(16),
        .TX_FIFO_DEPTH(16)
    ) u_mux (
        .clk         (clk),
        .rst_n       (rst_n),
        .data_in     (data_in),
        .sync_in     ({8{sync_in}}),
        .next_amps_in({8{nxt_amps}}),
        .out_valid   (out_valid),
        .out_word    (out_word),
        .out_chsel   (out_chsel)
    );

    // =========================================================================
    // DUT: 4 lane_framers  (parameters and SYNC words match top.v section 6)
    // =========================================================================
    lane_framer #(.SYNC_WORD(12'hA35), .LANE_NUM(2'd0)) u_f0 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(out_valid[0]), .in_word(out_word[0  +: 12]),
        .serial_out(serial_lane[0]));

    lane_framer #(.SYNC_WORD(12'hB46), .LANE_NUM(2'd1)) u_f1 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(out_valid[1]), .in_word(out_word[12 +: 12]),
        .serial_out(serial_lane[1]));

    lane_framer #(.SYNC_WORD(12'hC57), .LANE_NUM(2'd2)) u_f2 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(out_valid[2]), .in_word(out_word[24 +: 12]),
        .serial_out(serial_lane[2]));

    lane_framer #(.SYNC_WORD(12'hD68), .LANE_NUM(2'd3)) u_f3 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(out_valid[3]), .in_word(out_word[36 +: 12]),
        .serial_out(serial_lane[3]));

    // =========================================================================
    // Checker bookkeeping (module-level so they persist across always blocks)
    // =========================================================================
    integer ch_ptr  [0:7];    // next expected-word index per channel
    integer err_cnt   = 0;
    integer check_cnt = 0;

    // 12-bit shift register per lane for SYNC word detection
    reg [11:0] ser_sr    [0:3];
    integer    sync_hits [0:3];

    // =========================================================================
    // Helper functions
    // =========================================================================

    // Return expected word for a given channel and sequence index
    function automatic [11:0] get_expected;
        input integer ch;
        input integer ptr;
        case (ch)
            0: get_expected = exp_ch0[ptr];
            1: get_expected = exp_ch1[ptr];
            2: get_expected = exp_ch2[ptr];
            3: get_expected = exp_ch3[ptr];
            4: get_expected = exp_ch4[ptr];
            5: get_expected = exp_ch5[ptr];
            6: get_expected = exp_ch6[ptr];
            7: get_expected = exp_ch7[ptr];
            default: get_expected = 12'hX;
        endcase
    endfunction

    // SYNC word per lane (matches lane_framer parameters)
    function automatic [11:0] sync_word_for;
        input integer ln;
        case (ln)
            0: sync_word_for = 12'hA35;
            1: sync_word_for = 12'hB46;
            2: sync_word_for = 12'hC57;
            3: sync_word_for = 12'hD68;
            default: sync_word_for = 12'h000;
        endcase
    endfunction

    // =========================================================================
    // Initialise: load hex files and zero bookkeeping arrays
    // =========================================================================
    integer _i;
    initial begin
        $readmemh("sim/stim_h5.hex",       stim_mem);
        $readmemh("sim/expected_ch0.hex",  exp_ch0);
        $readmemh("sim/expected_ch1.hex",  exp_ch1);
        $readmemh("sim/expected_ch2.hex",  exp_ch2);
        $readmemh("sim/expected_ch3.hex",  exp_ch3);
        $readmemh("sim/expected_ch4.hex",  exp_ch4);
        $readmemh("sim/expected_ch5.hex",  exp_ch5);
        $readmemh("sim/expected_ch6.hex",  exp_ch6);
        $readmemh("sim/expected_ch7.hex",  exp_ch7);

        for (_i = 0; _i < 8; _i = _i + 1) ch_ptr[_i]    = 0;
        for (_i = 0; _i < 4; _i = _i + 1) begin
            ser_sr[_i]    = 12'h0;
            sync_hits[_i] = 0;
        end
    end

    // =========================================================================
    // Serial CSV dump  (clock + 4 serial lanes at every clock edge)
    // receiver.py reads this to decode lane_framer frames end-to-end.
    // =========================================================================
    integer csv_fd;
    initial csv_fd = $fopen("sim/serial_sim.csv", "w");
    initial $fwrite(csv_fd,
        "Time (s),la_clk,la_data_0,la_data_1,la_data_2,la_data_3\n");

    always @(clk)
        if (rst_n)
            $fwrite(csv_fd, "%.12e,%0d,%0d,%0d,%0d,%0d\n",
                    $realtime * 1e-9,
                    clk,
                    serial_lane[0], serial_lane[1], serial_lane[2], serial_lane[3]);

    // =========================================================================
    // Stimulus driver + final report
    // =========================================================================
    integer stim_idx;
    initial begin
        $dumpfile("sim/tb_h5.vcd");
        // Dump DUT I/O + submodule internals; use depth=1 for a smaller file
        $dumpvars(0, u_mux);
        $dumpvars(0, u_f0);
        $dumpvars(0, u_f1);
        $dumpvars(0, u_f2);
        $dumpvars(0, u_f3);

        // ── Reset ────────────────────────────────────────────────────────────
        rst_n    = 1'b0;
        data_in  = 8'h0;
        nxt_amps = 1'b0;
        sync_in  = 1'b0;
        repeat (8) @(posedge clk);
        rst_n = 1'b1;

        // ── Drive stimulus on negedge; DUT samples on the following posedge ──
        for (stim_idx = 0; stim_idx < N_CYCLES; stim_idx = stim_idx + 1) begin
            @(negedge clk);
            data_in  = stim_mem[stim_idx][7:0];
            nxt_amps = stim_mem[stim_idx][8];
            sync_in  = stim_mem[stim_idx][9];
        end

        // ── Drain pipeline: 2500 clocks covers remaining lane_framer emission ──
        repeat (2500) @(posedge clk);

        // ── Summary report ────────────────────────────────────────────────────
        $display("");
        $display("=============================================================");
        $display("rx_process_mux output words checked : %0d", check_cnt);
        $display("Mismatches                          : %0d", err_cnt);
        $display("lane_framer SYNC detections:");
        $display("  Lane 0 (0xA35) : %0d", sync_hits[0]);
        $display("  Lane 1 (0xB46) : %0d", sync_hits[1]);
        $display("  Lane 2 (0xC57) : %0d", sync_hits[2]);
        $display("  Lane 3 (0xD68) : %0d", sync_hits[3]);
        if (err_cnt == 0 &&
            sync_hits[0] >= 1 && sync_hits[1] >= 1 &&
            sync_hits[2] >= 1 && sync_hits[3] >= 1)
            $display("RESULT: PASS");
        else if (err_cnt == 0)
            $display("RESULT: PARTIAL PASS (no word errors; check sync_hits)");
        else
            $display("RESULT: FAIL  (%0d word mismatches)", err_cnt);
        $display("=============================================================");
        $fclose(csv_fd);
        $finish;
    end

    // =========================================================================
    // Output word checker
    // Fires every posedge; for each lane with out_valid, determines the channel
    // (lane*2 for even, lane*2+1 for odd) from out_chsel, then compares the
    // output word against the next expected value for that channel.
    // =========================================================================
    integer _lane, _ch;
    reg [11:0] _got, _exp;

    always @(posedge clk) begin
        for (_lane = 0; _lane < 4; _lane = _lane + 1) begin
            if (out_valid[_lane]) begin
                _ch  = _lane * 2 + (out_chsel[_lane] ? 1 : 0);
                _got = out_word[_lane * 12 +: 12];

                if (ch_ptr[_ch] < EXP_PER_CH) begin
                    _exp = get_expected(_ch, ch_ptr[_ch]);
                    check_cnt = check_cnt + 1;

                    if (_got !== _exp) begin
                        err_cnt = err_cnt + 1;
                        $display("[%0t ns] MISMATCH lane%0d ch%0d word[%0d]: exp=0x%03h got=0x%03h",
                                 $time, _lane, _ch, ch_ptr[_ch], _exp, _got);
                    end

                    ch_ptr[_ch] = ch_ptr[_ch] + 1;
                end
            end
        end
    end

    // =========================================================================
    // Serial SYNC scanner
    // Shifts serial_lane[ln] into a 12-bit SR each clock (MSB-first stream, so
    // ser_sr[11:0] accumulates the most-recently-received 12-bit word).
    // A match against the per-lane SYNC word is logged as a detection.
    // =========================================================================
    integer _ln;
    reg [11:0] _acc;

    always @(posedge clk) begin
        for (_ln = 0; _ln < 4; _ln = _ln + 1) begin
            _acc = {ser_sr[_ln][10:0], serial_lane[_ln]};
            ser_sr[_ln] <= _acc;
            if (_acc == sync_word_for(_ln)) begin
                sync_hits[_ln] <= sync_hits[_ln] + 1;
                $display("[%0t ns] SYNC 0x%03h on lane %0d",
                         $time, _acc, _ln);
            end
        end
    end

    // =========================================================================
    // Safety timeout
    // =========================================================================
    initial begin
        #(CLK_PERIOD * (N_CYCLES + 3000));
        $display("TIMEOUT at %0t ns -- stimulus or drain took too long", $time);
        $finish;
    end

endmodule
