// =============================================================================
// bit_packer.v — Pack 4 serial lane bits into 16-bit FIFO words
//
// Samples serial_out[3:0] every clock cycle. Every 4 clocks, writes a
// packed 16-bit word to the async FIFO.
//
// Word layout (MSB first in time):
//   [15:12] = time t0: {lane3, lane2, lane1, lane0}
//   [11:8]  = time t1: {lane3, lane2, lane1, lane0}
//   [7:4]   = time t2: {lane3, lane2, lane1, lane0}
//   [3:0]   = time t3: {lane3, lane2, lane1, lane0}
//
// Write rate: clk/4 words (e.g. 32 MHz → 8 MW/s = 16 MB/s)
// =============================================================================

module bit_packer (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,       // gate: don't write until system ready
    input  wire [3:0]  serial_in,    // 4 lane serial bits
    output reg         wen,          // FIFO write enable
    output reg  [15:0] wdata,        // FIFO write data
    input  wire        full          // FIFO full flag (back-pressure)
);

    reg [1:0]  cnt = 2'd0;
    reg [15:0] shift = 16'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt   <= 2'd0;
            shift <= 16'd0;
            wen   <= 1'b0;
            wdata <= 16'd0;
        end else if (!enable) begin
            cnt   <= 2'd0;
            shift <= 16'd0;
            wen   <= 1'b0;
        end else begin
            // Shift in current sample (t0 ends up in [15:12])
            shift <= {shift[11:0], serial_in};
            cnt   <= cnt + 1'b1;

            if (cnt == 2'd3) begin
                // 4 samples collected — write packed word
                wen   <= ~full;
                wdata <= {shift[11:0], serial_in};
            end else begin
                wen <= 1'b0;
            end
        end
    end

endmodule
