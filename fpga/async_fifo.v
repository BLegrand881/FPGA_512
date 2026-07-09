// =============================================================================
// async_fifo.v  —  Dual-clock asynchronous FIFO
//
// Based on Cummings' "Simulation and Synthesis Techniques for Asynchronous
// FIFO Design" (SNUG 2002).  Gray-coded pointers for safe CDC.
//
// Parameters:
//   W — data width (bits)
//   D — FIFO depth (must be power of 2)
// =============================================================================

module async_fifo #(
    parameter W = 16,
    parameter D = 1024
)(
    // Write side (producer clock)
    input  wire         wclk,
    input  wire         wrst_n,
    input  wire         wen,
    input  wire [W-1:0] wdata,
    output wire         full,

    // Read side (consumer clock)
    input  wire         rclk,
    input  wire         rrst_n,
    input  wire         ren,
    output wire [W-1:0] rdata,
    output wire [W-1:0] rdata_next,     // look-ahead: mem[rptr+1]
    output wire         empty,
    output wire         almost_empty    // will be empty after 1 more read
);

    localparam AW = $clog2(D);

    // Memory
    reg [W-1:0] mem [0:D-1];

    // Write-side pointers (binary and Gray)
    reg [AW:0] wptr_bin = 0;
    wire [AW:0] wptr_gray = wptr_bin ^ (wptr_bin >> 1);

    // Read-side pointers (binary and Gray)
    reg [AW:0] rptr_bin = 0;
    wire [AW:0] rptr_gray = rptr_bin ^ (rptr_bin >> 1);

    // Synchronized pointers (Gray-coded, 2-stage sync)
    reg [AW:0] wptr_gray_r1 = 0, wptr_gray_r2 = 0;  // wptr synced to rclk
    reg [AW:0] rptr_gray_r1 = 0, rptr_gray_r2 = 0;  // rptr synced to wclk

    // -----------------------------------------------------------------------
    // Write logic
    // -----------------------------------------------------------------------
    wire w_en = wen & ~full;

    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n)
            wptr_bin <= 0;
        else if (w_en)
            wptr_bin <= wptr_bin + 1;
    end

    always @(posedge wclk) begin
        if (w_en)
            mem[wptr_bin[AW-1:0]] <= wdata;
    end

    // Sync rptr (Gray) into write domain
    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) begin
            rptr_gray_r1 <= 0;
            rptr_gray_r2 <= 0;
        end else begin
            rptr_gray_r1 <= rptr_gray;
            rptr_gray_r2 <= rptr_gray_r1;
        end
    end

    // Full: MSBs differ, rest equal (in Gray code)
    assign full = (wptr_gray == {~rptr_gray_r2[AW:AW-1], rptr_gray_r2[AW-2:0]});

    // -----------------------------------------------------------------------
    // Read logic
    // -----------------------------------------------------------------------
    wire r_en = ren & ~empty;

    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n)
            rptr_bin <= 0;
        else if (r_en)
            rptr_bin <= rptr_bin + 1;
    end

    assign rdata = mem[rptr_bin[AW-1:0]];

    // Look-ahead: next word after current read pointer
    assign rdata_next = mem[rptr_bin[AW-1:0] + 1'b1];

    // Almost empty: FIFO will be empty after one more read
    wire [AW:0] rptr_next_bin  = rptr_bin + 1'b1;
    wire [AW:0] rptr_next_gray = rptr_next_bin ^ (rptr_next_bin >> 1);
    assign almost_empty = (rptr_next_gray == wptr_gray_r2);

    // Sync wptr (Gray) into read domain
    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) begin
            wptr_gray_r1 <= 0;
            wptr_gray_r2 <= 0;
        end else begin
            wptr_gray_r1 <= wptr_gray;
            wptr_gray_r2 <= wptr_gray_r1;
        end
    end

    // Empty: pointers equal (in Gray code)
    assign empty = (rptr_gray == wptr_gray_r2);

endmodule
