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

endmodule