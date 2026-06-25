`timescale 1ns/1ps

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

endmodule