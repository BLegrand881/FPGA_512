// =============================================================================
// spi_trigger.v — Button-triggered SPI initialization sequence
//
// Extracted from spi/top.v. On active-low button press, generates 4096 SPI
// clock cycles at clk_freq/CLK_DIVIDER, then pulses latch for one SPI period.
// Allows re-triggering after button release.
//
// States: IDLE → RUNNING (4096 clocks) → LATCH (1 period) → DONE → IDLE
// =============================================================================

module spi_trigger #(
    parameter CLK_DIVIDER = 16   // SPI_CLK = clk / CLK_DIVIDER (must be even, >= 2)
)(
    input  wire clk,
    input  wire btn_n,           // active-low button input
    output wire spi_sig1,        // data line 1 (high idle, low after trigger)
    output wire spi_sig2,        // data line 2 (high idle, low after trigger)
    output wire spi_clk_o,       // SPI clock output
    output wire spi_latch,       // latch pulse (1 SPI period after clocks)
    output wire done             // latched high after first SPI sequence completes
);

    localparam ST_IDLE    = 2'b00;
    localparam ST_RUNNING = 2'b01;
    localparam ST_LATCH   = 2'b10;
    localparam ST_DONE    = 2'b11;

    // Startup delay: ignore button for ~100 ms after power-up
    // 16 MHz × 2^21 ≈ 131 ms
    reg [20:0] startup_ctr = 21'd0;
    wire startup_done = startup_ctr[20];
    always @(posedge clk)
        if (!startup_done)
            startup_ctr <= startup_ctr + 1'b1;

    reg [1:0]  state               = ST_IDLE;
    reg [7:0]  clk_half_period_ctr = 0;
    reg        latch_phase         = 1'b0;
    reg [11:0] spi_cycle_ctr       = 0;

    reg out_sig1  = 1'b1;
    reg out_sig2  = 1'b1;
    reg out_clk   = 1'b0;
    reg out_latch = 1'b0;

    // Latch: once SPI completes, stays high forever
    reg spi_complete = 1'b0;

    always @(posedge clk) begin
        case (state)

            ST_IDLE: begin
                out_latch   <= 1'b0;
                latch_phase <= 1'b0;

                if (startup_done && btn_n == 1'b0) begin
                    out_sig1            <= 1'b0;
                    out_sig2            <= 1'b0;
                    out_clk             <= 1'b0;
                    clk_half_period_ctr <= 0;
                    spi_cycle_ctr       <= 0;
                    state               <= ST_RUNNING;
                end
            end

            ST_RUNNING: begin
                if (clk_half_period_ctr >= (CLK_DIVIDER / 2) - 1) begin
                    clk_half_period_ctr <= 0;

                    if (out_clk == 1'b1) begin
                        out_clk <= 1'b0;

                        if (spi_cycle_ctr == 12'hFFF) begin
                            out_latch   <= 1'b1;
                            latch_phase <= 1'b0;
                            state       <= ST_LATCH;
                        end else begin
                            spi_cycle_ctr <= spi_cycle_ctr + 1'b1;
                        end
                    end else begin
                        out_clk <= 1'b1;
                    end
                end else begin
                    clk_half_period_ctr <= clk_half_period_ctr + 1'b1;
                end
            end

            ST_LATCH: begin
                out_clk <= 1'b0;

                if (clk_half_period_ctr >= (CLK_DIVIDER / 2) - 1) begin
                    clk_half_period_ctr <= 0;

                    if (latch_phase == 1'b1) begin
                        out_latch    <= 1'b0;
                        spi_complete <= 1'b1;
                        state        <= ST_DONE;
                    end else begin
                        latch_phase <= 1'b1;
                    end
                end else begin
                    clk_half_period_ctr <= clk_half_period_ctr + 1'b1;
                end
            end

            ST_DONE: begin
                out_latch <= 1'b0;
                out_sig1  <= 1'b0;
                out_sig2  <= 1'b0;
                out_clk   <= 1'b0;

                if (btn_n == 1'b1)
                    state <= ST_IDLE;
            end

            default: state <= ST_IDLE;
        endcase
    end

    assign spi_sig1  = out_sig1;
    assign spi_sig2  = out_sig2;
    assign spi_clk_o = out_clk;
    assign spi_latch = out_latch;
    assign done      = spi_complete;

endmodule
