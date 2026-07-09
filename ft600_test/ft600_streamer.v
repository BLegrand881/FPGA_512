// =============================================================================
// ft600_streamer.v  —  Counter streamer for FT600 link validation
//
// Streams an incrementing 16-bit counter to the FT600 whenever TXE_N is low.
// Uses confirmed-acceptance gating: the counter only advances when the FT600
// has confirmed capture (WR_N=0 AND TXE_N=0 on the same edge).  A combinational
// bypass (counter+1) keeps data_out one-ahead so no settle cycle is needed.
//
// This eliminates the 1-skip-per-2048-words error at buffer boundaries while
// maintaining full-speed (1 word per clock) throughput.
// =============================================================================

module ft600_streamer (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        txe_n,
    output reg         wr_n,
    output reg         rd_n,
    output reg         oe_n,
    output reg  [1:0]  be,
    output reg  [15:0] data_out,
    output reg         data_oe
);

    reg [15:0] counter;

    // FT600 captured our data on THIS clock edge
    wire accepted = ~wr_n & ~txe_n;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter  <= 16'd0;
            wr_n     <= 1'b1;
            rd_n     <= 1'b1;
            oe_n     <= 1'b1;
            be       <= 2'b00;
            data_out <= 16'd0;
            data_oe  <= 1'b0;
        end else begin
            rd_n <= 1'b1;
            oe_n <= 1'b1;

            if (!txe_n) begin
                wr_n     <= 1'b0;
                be       <= 2'b11;
                // Bypass: when accepted, the current data_out is being captured
                // right now, so drive counter+1 for the NEXT capture.  When not
                // accepted (startup or re-entry after TXE_N gap), drive counter.
                data_out <= accepted ? (counter + 1'b1) : counter;
                data_oe  <= 1'b1;
            end else begin
                wr_n     <= 1'b1;
                be       <= 2'b00;
                data_oe  <= 1'b0;
            end

            // Counter advances only on confirmed capture — never speculatively
            if (accepted)
                counter <= counter + 1'b1;
        end
    end

endmodule
