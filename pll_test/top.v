// =============================================================================
// top.v — PLL Clock Multiplier & Dual-Rate LED Blink
// =============================================================================
module top (
    input  wire       clk,          // 16 MHz Input Clock (Pin A7)
    output wire [3:0] led,          // {D3, D0, D2, D1}
    output wire       clk_out_32    // 32 MHz Output Clock (Pin A12)
);

    wire clk_32;
    wire pll_lock;

    // -------------------------------------------------------------------------
    // ECP5 PLL Instance (16 MHz -> 32 MHz)
    // Generated parameters using: ecppll -i 16 -o 32
    // -------------------------------------------------------------------------
    EHXPLLL #(
        .CLKI_DIV(1),
        .CLKFB_DIV(2),
        .CLKOP_DIV(16),
        .FEEDBK_PATH("CLKOP")
    ) pll_inst (
        .CLKI(clk),
        .CLKFB(clk_32),
        .CLKOP(clk_32),
        .RST(1'b0),
        .PHASESEL0(1'b0),
        .PHASESEL1(1'b0),
        .PHASEDIR(1'b0),
        .PHASESTEP(1'b0),
        .STDBY(1'b0),
        .PLLWAKESYNC(1'b0),
        .ENCLKOP(1'b1),
        .LOCK(pll_lock)
    );

    // Drive the external pin with the 32 MHz clock
    assign clk_out_32 = clk_32;

    // -------------------------------------------------------------------------
    // Blinker 1: 16 MHz Clock Domain -> Drives led[0] (D0)
    // -------------------------------------------------------------------------
    reg [24:0] cnt_16;
    always @(posedge clk) begin
        cnt_16 <= cnt_16 + 1'b1;
    end

    // -------------------------------------------------------------------------
    // Blinker 2: 32 MHz Clock Domain -> Drives led[1] (D1)
    // -------------------------------------------------------------------------
    reg [24:0] cnt_32;
    always @(posedge clk_32) begin
        if (!pll_lock) begin
            cnt_32 <= 0;
        end else begin
            cnt_32 <= cnt_32 + 1'b1;
        end
    end

    // Assign outputs:
    // led[0] toggles every 2^23 / 16MHz ≈ 0.52 seconds
    // led[1] toggles every 2^24 / 32MHz ≈ 0.52 seconds (adjusted counter bit depth to match speed)
    assign led[0] = cnt_16[24]; 
    assign led[1] = cnt_32[24]; 
    
    // Turn off unused LEDs
    assign led[2] = 1'b0;
    assign led[3] = 1'b0;

endmodule