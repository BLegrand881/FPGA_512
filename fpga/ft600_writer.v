// =============================================================================
// ft600_writer.v  —  Reads from async FIFO, writes to FT600 245 FIFO mode.
//
// Uses confirmed-acceptance gating with a combinational look-ahead bypass.
// The FIFO read pointer only advances when the FT600 confirms capture
// (WR_N=0 AND TXE_N=0).  On the same cycle, data_out is loaded from the
// FIFO's look-ahead port (rdata_next = mem[rptr+1]) so no settle cycle
// is needed.  Runs at full speed: 1 word per clock when TXE_N is low.
// =============================================================================

module ft600_writer (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        txe_n,
    output reg         wr_n,
    output reg         rd_n,
    output reg         oe_n,
    output reg  [1:0]  be,
    output reg  [15:0] data_out,
    output reg         data_oe,

    output wire        fifo_ren,
    input  wire [15:0] fifo_rdata,
    input  wire [15:0] fifo_rdata_next,
    input  wire        fifo_empty,
    input  wire        fifo_almost_empty
);

    // FT600 captured our data on THIS clock edge
    wire accepted = ~wr_n & ~txe_n;

    // Advance FIFO only when write confirmed
    assign fifo_ren = accepted;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_n     <= 1'b1;
            rd_n     <= 1'b1;
            oe_n     <= 1'b1;
            be       <= 2'b00;
            data_out <= 16'd0;
            data_oe  <= 1'b0;
        end else begin
            rd_n <= 1'b1;
            oe_n <= 1'b1;

            if (~fifo_empty & ~txe_n & ~(accepted & fifo_almost_empty)) begin
                // FIFO has data and FT600 can accept.
                // Guard: if accepted AND this is the last word, stop —
                // rdata_next would be invalid.
                wr_n     <= 1'b0;
                be       <= 2'b11;
                // Bypass: when accepted, current data_out is being captured
                // right now so drive the NEXT word via look-ahead port.
                // When not accepted (startup / re-entry), drive current word.
                data_out <= accepted ? fifo_rdata_next : fifo_rdata;
                data_oe  <= 1'b1;
            end else begin
                wr_n    <= 1'b1;
                be      <= 2'b00;
                data_oe <= 1'b0;
            end
        end
    end

endmodule
