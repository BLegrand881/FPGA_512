module adc_stream_gen #(
    parameter int N_AMPS       = 64,
    parameter int N_ADC_PER_GP = 4,
    parameter int ADC_BITS     = 12,
    parameter int ZERO_CYCLES  = 16,
    parameter int DATA_CYCLES  = 48,
    parameter int GROUP_CYCLES = 64,
    parameter int N_GROUPS     = 16,
    parameter int FRAME_CYCLES = 1024,
    parameter int TEST_ITERS   = 3
)(
    input  wire clk,
    input  wire rst_n,
    output reg  sync,
    output reg  next_amps,
    output reg  data
);

    // -------------------- storage --------------------
    reg [ADC_BITS-1:0] amp [0:TEST_ITERS-1][0:N_AMPS-1];

    integer t, i;
    initial begin
        for (t = 0; t < TEST_ITERS; t = t + 1)
            for (i = 0; i < N_AMPS; i = i + 1)
                amp[t][i] = $random & ((1<<ADC_BITS)-1);
    end

    // -------------------- state --------------------
    integer frame_cyc;

    // -------------------- combinational: derive next outputs --------------------
    // All blocking assigns. These are wires conceptually; we use reg only
    // because they're assigned inside an always block.
    reg        sync_n, next_amps_n, data_n;
    integer    group_idx, cyc_in_group;
    integer    bit_idx, which_adc, which_bit, amp_index;
    integer    frame_cyc_n;
    reg [$clog2(TEST_ITERS)-1:0] test_idx;


    always @* begin
        // position
        group_idx    = frame_cyc / GROUP_CYCLES;
        cyc_in_group = frame_cyc % GROUP_CYCLES;

        // sync + next_amps
        sync_n      = (frame_cyc < GROUP_CYCLES);
        next_amps_n = (cyc_in_group == 0);

        // data
        if (cyc_in_group < ZERO_CYCLES) begin
            bit_idx   = 0;
            which_adc = 0;
            which_bit = 0;
            amp_index = 0;
            data_n    = 1'b0;
        end else begin
            bit_idx   = cyc_in_group - ZERO_CYCLES;
            which_adc = bit_idx % N_ADC_PER_GP;
            which_bit = bit_idx / N_ADC_PER_GP;
            amp_index = N_ADC_PER_GP * group_idx + which_adc;
            data_n    = amp[test_idx][amp_index][which_bit];
        end

        // counter
        frame_cyc_n = (frame_cyc == FRAME_CYCLES-1) ? 0 : frame_cyc + 1;

        if (frame_cyc == FRAME_CYCLES-1)
            test_idx_n = (test_idx == TEST_ITERS-1) ? 0 : test_idx + 1;
        else
            test_idx_n = test_idx;
    end

    // -------------------- sequential: register outputs --------------------
    always @(negedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_cyc <= 0a
            sync      <= 1'b0;
            next_amps <= 1'b0;
            data      <= 1'b0;
            test_idx <= 0;
        end else begin
            frame_cyc <= frame_cyc_n;
            sync      <= sync_n;
            next_amps <= next_amps_n;
            data      <= data_n;
            test_idx <= test_idx_n;
        end
    end

endmodule`timescale 1ns/1ps

module tb_adc_stream;

    // ---------------- params (match the DUT/TX) ----------------
    localparam int N_AMPS       = 64;
    localparam int N_ADC_PER_GP = 4;
    localparam int ADC_BITS     = 12;
    localparam int ZERO_CYCLES  = 16;
    localparam int GROUP_CYCLES = 64;
    localparam int N_GROUPS     = 16;
    localparam int FRAME_CYCLES = 1024;
    localparam int TEST_ITERS   = 3;

    localparam time CLK_PERIOD  = 31.25ns;  // ~32 MHz

    // ---------------- DUT/TX wiring ----------------
    reg  clk = 0;
    reg  rst_n = 0;
    wire sync, next_amps, data;
    wire [ADC_BITS-1:0] amp_out [0:N_AMPS-1];
    wire frame_done;

    // clock
    always #(CLK_PERIOD/2) clk = ~clk;

    // ---------------- instantiate ----------------
    adc_stream_gen #(
        .N_AMPS(N_AMPS), .N_ADC_PER_GP(N_ADC_PER_GP), .ADC_BITS(ADC_BITS),
        .ZERO_CYCLES(ZERO_CYCLES), .GROUP_CYCLES(GROUP_CYCLES),
        .N_GROUPS(N_GROUPS), .FRAME_CYCLES(FRAME_CYCLES),
        .TEST_ITERS(TEST_ITERS)
    ) u_tx (
        .clk(clk), .rst_n(rst_n),
        .sync(sync), .next_amps(next_amps), .data(data)
    );

    adc_rx #(
        .N_AMPS(N_AMPS), .N_ADC_PER_GP(N_ADC_PER_GP), .ADC_BITS(ADC_BITS),
        .ZERO_CYCLES(ZERO_CYCLES), .GROUP_CYCLES(GROUP_CYCLES),
        .N_GROUPS(N_GROUPS)
    ) u_rx (
        .clk(clk), .rst_n(rst_n),
        .sync(sync), .next_amps(next_amps), .data(data),
        .amp_out(amp_out), .frame_done(frame_done)
    );

    // ---------------- bookkeeping ----------------
    integer errors    = 0;
    integer compares  = 0;
    integer frames_checked = 0;
    integer t;  // which test iter we expect to have just finished

    // ---------------- stimulus ----------------
    initial begin
        $dumpfile("tb_adc_stream.vcd");
        $dumpvars(0, tb_adc_stream);

        // reset
        rst_n = 0;
        repeat (4) @(posedge clk);
        rst_n = 1;

        // run all TEST_ITERS frames + a bit of slack
        for (t = 0; t < TEST_ITERS; t = t + 1) begin
            // wait for end-of-frame pulse from DUT
            @(posedge clk iff frame_done);
            // one delta later, amp_out is settled
            #1;
            check_frame(t);
            frames_checked = frames_checked + 1;
        end

        // summary
        $display("--------------------------------------------------");
        $display("Frames checked : %0d / %0d", frames_checked, TEST_ITERS);
        $display("Comparisons    : %0d", compares);
        $display("Errors         : %0d", errors);
        if (errors == 0) $display("RESULT: PASS");
        else             $display("RESULT: FAIL");
        $display("--------------------------------------------------");
        $finish;
    end

    // ---------------- checker ----------------
    task check_frame(input integer iter);
        integer i;
        reg [ADC_BITS-1:0] exp, got;
        begin
            for (i = 0; i < N_AMPS; i = i + 1) begin
                exp = u_tx.amp[iter][i];   // hierarchical peek into TX
                got = amp_out[i];
                compares = compares + 1;
                if (exp !== got) begin
                    errors = errors + 1;
                    $display("[t=%0t] MISMATCH iter=%0d amp[%0d]: exp=0x%0h got=0x%0h",
                             $time, iter, i, exp, got);
                end
            end
            $display("[t=%0t] frame %0d checked (%0d amps)", $time, iter, N_AMPS);
        end
    endtask

    // ---------------- safety timeout ----------------
    initial begin
        #(CLK_PERIOD * FRAME_CYCLES * (TEST_ITERS + 2));
        $display("TIMEOUT: simulation ran too long");
        $display("Errors so far: %0d", errors);
        $finish;
    end

endmodulemodule adc_rx #(
    parameter int N_AMPS       = 64,
    parameter int N_ADC_PER_GP = 4,
    parameter int ADC_BITS     = 12,
    parameter int ZERO_CYCLES  = 16,
    parameter int GROUP_CYCLES = 64,
    parameter int N_GROUPS     = 16
)(
    input  wire clk,
    input  wire rst_n,
    input  wire sync,
    input  wire next_amps,
    input  wire data,
    output reg  [ADC_BITS-1:0] amp_out [0:N_AMPS-1],
    output reg  frame_done   // 1-cycle pulse when amp_out is fully valid
);
    // Mirror of TX bookkeeping. Sample on posedge — TX drives on negedge,
    // so data is stable here.
    integer frame_cyc, group_idx, cyc_in_group;
    integer bit_idx, which_adc, which_bit, amp_index;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_cyc  <= 0;
            frame_done <= 1'b0;
        end else begin
            // resync on sync high at frame start
            if (sync && frame_cyc >= GROUP_CYCLES)
                frame_cyc <= 1;   // we just consumed cycle 0
            else
                frame_cyc <= (frame_cyc == N_GROUPS*GROUP_CYCLES-1) ? 0 : frame_cyc + 1;

            group_idx    = frame_cyc / GROUP_CYCLES;
            cyc_in_group = frame_cyc % GROUP_CYCLES;

            if (cyc_in_group >= ZERO_CYCLES) begin
                bit_idx   = cyc_in_group - ZERO_CYCLES;
                which_adc = bit_idx % N_ADC_PER_GP;
                which_bit = bit_idx / N_ADC_PER_GP;
                amp_index = N_ADC_PER_GP * group_idx + which_adc;
                amp_out[amp_index][which_bit] <= data;
            end

            // pulse frame_done one cycle after the last data bit lands
            frame_done <= (frame_cyc == N_GROUPS*GROUP_CYCLES-1);
        end
    end
endmodule