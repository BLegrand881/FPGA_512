`timescale 1ns/1ps

// =====================================================================
//  TX: serial stream generator
// =====================================================================
module adc_stream_gen #(
    parameter integer N_AMPS       = 64,
    parameter integer N_ADC_PER_GP = 4,
    parameter integer ADC_BITS     = 12,
    parameter integer ZERO_CYCLES  = 16,
    parameter integer DATA_CYCLES  = 48,
    parameter integer GROUP_CYCLES = 64,
    parameter integer N_GROUPS     = 16,
    parameter integer FRAME_CYCLES = 1024,
    parameter integer TEST_ITERS   = 3
)(
    input  wire clk,
    input  wire rst_n,
    output reg  sync,
    output reg  next_amps,
    output reg  data
);

    // -------------------- storage --------------------
    reg [ADC_BITS-1:0] amp [0:TEST_ITERS-1][0:N_AMPS-1];

    integer t_init, i_init;
    initial begin
        for (t_init = 0; t_init < TEST_ITERS; t_init = t_init + 1)
            for (i_init = 0; i_init < N_AMPS; i_init = i_init + 1)
                amp[t_init][i_init] = $random & ((1<<ADC_BITS)-1);
    end

    // -------------------- state --------------------
    integer frame_cyc;
    reg [31:0] test_idx;     // generous width; trivial

    // -------------------- combinational next-state --------------------
    reg        sync_n, next_amps_n, data_n;
    integer    group_idx, cyc_in_group;
    integer    bit_idx, which_adc, which_bit, amp_index;
    integer    frame_cyc_n;
    reg [31:0] test_idx_n;

    always @* begin
        group_idx    = frame_cyc / GROUP_CYCLES;
        cyc_in_group = frame_cyc % GROUP_CYCLES;

        sync_n      = (frame_cyc < GROUP_CYCLES);
        next_amps_n = (cyc_in_group == 0);

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

        frame_cyc_n = (frame_cyc == FRAME_CYCLES-1) ? 0 : frame_cyc + 1;

        if (frame_cyc == FRAME_CYCLES-1)
            test_idx_n = (test_idx == TEST_ITERS-1) ? 0 : test_idx + 1;
        else
            test_idx_n = test_idx;
    end

    // -------------------- sequential: register outputs --------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_cyc <= 0;
            sync      <= 1'b0;
            next_amps <= 1'b0;
            data      <= 1'b0;
            test_idx  <= 0;
        end else begin
            frame_cyc <= frame_cyc_n;
            sync      <= sync_n;
            next_amps <= next_amps_n;
            data      <= data_n;
            test_idx  <= test_idx_n;
        end
    end

endmodule


// =====================================================================
//  RX: deserialize the stream
// =====================================================================
module adc_rx #(
    parameter integer N_AMPS       = 64,
    parameter integer N_ADC_PER_GP = 4,
    parameter integer ADC_BITS     = 12,
    parameter integer ZERO_CYCLES  = 16,
    parameter integer GROUP_CYCLES = 64,
    parameter integer N_GROUPS     = 16
)(
    input  wire clk,
    input  wire rst_n,
    input  wire sync,
    input  wire next_amps,
    input  wire data,
    output reg  [ADC_BITS-1:0] amp_out [0:N_AMPS-1],
    output reg  frame_done
);
    // Strategy: a per-group cycle counter (0..GROUP_CYCLES-1) driven by
    // next_amps rising edge to reset, and a group index driven by next_amps
    // count. Frame start = sync rising edge.
    integer cyc_in_group;
    integer group_idx;
    integer bit_idx, which_adc, which_bit, amp_index;
    reg     next_amps_d, sync_d;
    integer k_init;

    initial begin
        for (k_init = 0; k_init < N_AMPS; k_init = k_init + 1)
            amp_out[k_init] = 0;
    end

    always @(negedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cyc_in_group <= 0;
            group_idx    <= 0;
            next_amps_d  <= 1'b0;
            sync_d       <= 1'b0;
            frame_done   <= 1'b0;
        end else begin
            next_amps_d <= next_amps;
            sync_d      <= sync;

            // On next_amps rising edge: reset cyc_in_group, advance group.
            // On sync rising edge: also reset group_idx to 0.
            if (sync & ~sync_d) begin
                group_idx    <= 0;
                cyc_in_group <= 1;   // we ARE consuming cycle 0 right now
            end else if (next_amps & ~next_amps_d) begin
                group_idx    <= group_idx + 1;
                cyc_in_group <= 1;
            end else begin
                cyc_in_group <= cyc_in_group + 1;
            end

            // Sample data when inside the data window of the current group.
            // Use combinational current cyc_in_group (not the about-to-update).
            if (cyc_in_group >= ZERO_CYCLES && cyc_in_group < GROUP_CYCLES) begin
                bit_idx   = cyc_in_group - ZERO_CYCLES;
                which_adc = bit_idx % N_ADC_PER_GP;
                which_bit = bit_idx / N_ADC_PER_GP;
                amp_index = N_ADC_PER_GP * group_idx + which_adc;
                amp_out[amp_index][which_bit] <= data;
            end

            // Pulse frame_done at the end of the last group of the frame.
            frame_done <= (group_idx == N_GROUPS-1) &&
                          (cyc_in_group == GROUP_CYCLES-1);
        end
    end
endmodule


// =====================================================================
//  Testbench top
// =====================================================================
module tb_adc_stream;

    localparam integer N_AMPS       = 64;
    localparam integer N_ADC_PER_GP = 4;
    localparam integer ADC_BITS     = 12;
    localparam integer ZERO_CYCLES  = 16;
    localparam integer GROUP_CYCLES = 64;
    localparam integer N_GROUPS     = 16;
    localparam integer FRAME_CYCLES = 1024;
    localparam integer TEST_ITERS   = 3;

    // 32 MHz -> 31.25 ns period. iverilog handles real delays fine.
    localparam real CLK_PERIOD = 31.25;

    reg  clk = 0;
    reg  rst_n = 0;
    wire sync, next_amps, data;
    wire [ADC_BITS-1:0] amp_out [0:N_AMPS-1];
    wire frame_done;

    always #(CLK_PERIOD/2) clk = ~clk;

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

    integer errors         = 0;
    integer compares       = 0;
    integer frames_checked = 0;
    integer t;

    // Wait helper instead of @(posedge clk iff ...) which iverilog dislikes
    task wait_frame_done;
        begin
            @(posedge clk);
            while (!frame_done) @(posedge clk);
        end
    endtask

    task check_frame(input integer iter);
        integer i;
        reg [ADC_BITS-1:0] exp_v, got_v;
        begin
            for (i = 0; i < N_AMPS; i = i + 1) begin
                exp_v = u_tx.amp[iter][i];
                got_v = amp_out[i];
                compares = compares + 1;
                if (exp_v !== got_v) begin
                    errors = errors + 1;
                    if (errors <= 20)
                        $display("[t=%0t] MISMATCH iter=%0d amp[%0d]: exp=0x%0h got=0x%0h",
                                 $time, iter, i, exp_v, got_v);
                end
            end
            $display("[t=%0t] frame %0d checked (%0d amps, errors so far=%0d)",
                     $time, iter, N_AMPS, errors);
        end
    endtask

    initial begin
        $dumpfile("tb_adc_stream.vcd");
        $dumpvars(0, tb_adc_stream);

        rst_n = 0;
        repeat (4) @(posedge clk);
        rst_n = 1;

        for (t = 0; t < TEST_ITERS; t = t + 1) begin
            wait_frame_done();
            #1;
            check_frame(t);
            frames_checked = frames_checked + 1;
        end

        $display("--------------------------------------------------");
        $display("Frames checked : %0d / %0d", frames_checked, TEST_ITERS);
        $display("Comparisons    : %0d", compares);
        $display("Errors         : %0d", errors);
        if (errors == 0) $display("RESULT: PASS");
        else             $display("RESULT: FAIL");
        $display("--------------------------------------------------");
        $finish;
    end

    initial begin
        #(CLK_PERIOD * FRAME_CYCLES * (TEST_ITERS + 2));
        $display("TIMEOUT");
        $display("Errors so far: %0d", errors);
        $finish;
    end

endmodule
