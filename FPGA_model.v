module adc_rx #(
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