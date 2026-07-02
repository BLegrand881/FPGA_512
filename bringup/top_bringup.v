// =============================================================================
// top_bringup.v  —  Bringup loopback for custom FPGA board (LFE5U-25F BGA256)
//
// Reflects 4 signals from CN1 (ADC board interface) directly to J6 pins 3-6,
// with the ADC clock also on J6 pin 7, for scope/LA verification.
//
// J6 output mapping:
//   Pin 3  A15  CB_D[1]      — data lane 1 (raw serial)
//   Pin 4  A14  CB_READ      — next_amps group-boundary pulse
//   Pin 5  B14  CB_SYNC      — frame sync
//   Pin 6  A13  CB_CLK32MHZ  — 32 MHz ADC bit clock
//   Pin 7  A12  CB_CLK32MHZ  — 32 MHz ADC bit clock (clock reference)
//
// No PLL, no decoder — pure combinational passthrough.
// =============================================================================

module top (
    input  wire        clk_16m,      // A7  — 16 MHz crystal (always present)

    // CN1 — ADC board inputs
    input  wire        cb_d1,        // B15 — serial data lane 1
    input  wire        cb_clk32mhz,  // K16 — 32 MHz bit clock from ADC board
    input  wire        cb_read,      // J13 — next_amps group-boundary pulse
    input  wire        cb_sync,      // H14 — frame sync

    // Status LED — blinks from crystal so you can confirm the bitstream loaded
    // without plugging in the ADC board
    output wire        led_power,    // P16 → D1: ~1 Hz blink from 16 MHz crystal

    // J6 — loopback outputs
    output wire        j6_d1,        // A15 — pin 3
    output wire        j6_read,      // A14 — pin 4
    output wire        j6_sync,      // B14 — pin 5
    output wire        j6_clk,       // A13 — pin 6
    output wire        j6_clkref     // A12 — pin 7
);

    // 24-bit counter at 16 MHz → toggles at 16e6/2^24 ≈ 0.95 Hz
    reg [23:0] ctr = 24'd0;
    always @(posedge clk_16m)
        ctr <= ctr + 1'b1;

    assign led_power = ctr[23];   // MSB = ~1 Hz blink

    assign j6_d1     = cb_d1;
    assign j6_read   = cb_read;
    assign j6_sync   = cb_sync;
    assign j6_clk    = cb_clk32mhz;
    assign j6_clkref = cb_clk32mhz;

endmodule
