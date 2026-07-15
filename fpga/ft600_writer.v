// =============================================================================
// ft600_writer.v  —  Bidirectional FT600 245 FIFO mode interface.
//
// Write path: confirmed-acceptance gating with combinational look-ahead bypass.
// Read path:  when RXF_N goes low (host sent data), briefly pauses writes,
//             reads one 16-bit command word, then resumes.  ~4 cycle gap.
// =============================================================================

module ft600_writer (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        txe_n,
    input  wire        rxf_n,
    output reg         wr_n,
    output reg         rd_n,
    output reg         oe_n,
    output reg  [1:0]  be,
    output reg  [15:0] data_out,
    output reg         data_oe,

    // Read-from-host command output
    input  wire [15:0] ft_data_in,
    output reg         cmd_valid,
    output reg  [15:0] cmd_data,

    output wire        fifo_ren,
    input  wire [15:0] fifo_rdata,
    input  wire [15:0] fifo_rdata_next,
    input  wire        fifo_empty,
    input  wire        fifo_almost_empty
);

    localparam S_WRITE          = 3'd0;
    localparam S_READ_TURNAROUND= 3'd1;
    localparam S_READ_OE        = 3'd2;
    localparam S_READ_LATCH     = 3'd3;
    localparam S_WRITE_RESUME   = 3'd4;

    reg [2:0] state = S_WRITE;

    // FT600 captured our data on THIS clock edge
    wire accepted = ~wr_n & ~txe_n;

    // Advance FIFO only when write confirmed
    assign fifo_ren = accepted;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_n      <= 1'b1;
            rd_n      <= 1'b1;
            oe_n      <= 1'b1;
            be        <= 2'b00;
            data_out  <= 16'd0;
            data_oe   <= 1'b0;
            cmd_valid <= 1'b0;
            cmd_data  <= 16'd0;
            state     <= S_WRITE;
        end else begin
            cmd_valid <= 1'b0;

            case (state)
            S_WRITE: begin
                if (~rxf_n) begin
                    // Host sent data — pause writes, start read sequence
                    wr_n    <= 1'b1;
                    be      <= 2'b00;
                    data_oe <= 1'b0;
                    state   <= S_READ_TURNAROUND;
                end else if (~fifo_empty & ~txe_n & ~(accepted & fifo_almost_empty)) begin
                    wr_n     <= 1'b0;
                    be       <= 2'b11;
                    data_out <= accepted ? fifo_rdata_next : fifo_rdata;
                    data_oe  <= 1'b1;
                end else begin
                    wr_n    <= 1'b1;
                    be      <= 2'b00;
                    data_oe <= 1'b0;
                end
            end

            S_READ_TURNAROUND: begin
                // Bus released, assert OE_N to let FT600 drive
                oe_n  <= 1'b0;
                state <= S_READ_OE;
            end

            S_READ_OE: begin
                // FT600 is driving the bus, assert RD_N
                rd_n  <= 1'b0;
                state <= S_READ_LATCH;
            end

            S_READ_LATCH: begin
                // Data valid on bus — latch it
                cmd_data  <= ft_data_in;
                cmd_valid <= 1'b1;
                rd_n      <= 1'b1;
                oe_n      <= 1'b1;
                state     <= S_WRITE_RESUME;
            end

            S_WRITE_RESUME: begin
                // Bus turnaround back to FPGA drive
                state <= S_WRITE;
            end

            default: state <= S_WRITE;
            endcase
        end
    end

endmodule
