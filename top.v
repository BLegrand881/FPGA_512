// =============================================================================
// top.v  —  ECP5-5G Eval Board, single-channel ADC serial loopback test
//
// Physical setup:
//   Jumper a wire from J39 Pin 5 (B15) -> J39 Pin 6 (C15)
//   Attach logic analyzer to any probed pins below
//
// J39 pins used:
//   Pin 5  (B15)  serial_out     — TX serial bitstream (probe / loopback source)
//   Pin 6  (C15)  serial_in      — RX serial bitstream (jumper from Pin 5)
//   Pin 7  (B13)  sync_out       — frame sync pulse     (probe)
//   Pin 8  (B20)  next_amps_out  — group-boundary pulse (probe)
//
// LEDs (active-low):
//   LED0  A13  — blinks ~1.9 Hz when frames are completing (healthy)
//   LED1  A12  — latches ON when watchdog fires (no frame_done in time)
//   LED2  B19  — toggles on every sync rising edge (TX heartbeat)
//   LED3  A18  — spare (off)
//
// Reset:
//   SW4  P4  — active-low push button; also held during PLL unlock
// =============================================================================

module top (
    input  wire clk_12m,        // 12 MHz FTDI clock, ball A10 (JP2 must be installed)
    input  wire rst_btn_n,      // SW4, ball P4, active-low

    // J39 expansion header
    output wire serial_out,     // J39 Pin 5  (B15)
    input  wire serial_in,      // J39 Pin 6  (C15) — jumper to Pin 5 for loopback
    output wire sync_out,       // J39 Pin 7  (B13)
    output wire next_amps_out,  // J39 Pin 8  (B20)

    // General-purpose LEDs (active-low)
    output wire led0,           // A13
    output wire led1,           // A12
    output wire led2,           // B19
    output wire led3            // A18
);

    // -------------------------------------------------------------------------
    // Clock  —  LEAVE BLANK, user supplies their own clock module.
    // Replace the two assigns below with your PLL / clock-module instantiation.
    // Expected outputs: clk_32 (32 MHz), pll_locked (high when stable).
    // -------------------------------------------------------------------------
    wire clk_32;
    wire pll_locked;

    // TODO: instantiate your clock module here
    assign clk_32    = clk_12m; // PLACEHOLDER — replace with PLL output
    assign pll_locked = 1'b1;   // PLACEHOLDER — replace with PLL locked signal

    // -------------------------------------------------------------------------
    // Reset: synchronous de-assertion, gated on PLL lock
    // -------------------------------------------------------------------------
    reg [3:0] rst_pipe = 4'hF;
    always @(posedge clk_32) begin
        if (!rst_btn_n || !pll_locked)
            rst_pipe <= 4'hF;
        else
            rst_pipe <= {rst_pipe[2:0], 1'b0};
    end
    wire rst_n = ~rst_pipe[3];

    // -------------------------------------------------------------------------
    // TX — serial stream generator (1 channel)
    // -------------------------------------------------------------------------
    wire tx_sync;
    wire tx_next_amps;
    wire tx_data;

    adc_stream_gen #(
        .N_AMPS      (64),
        .N_ADC_PER_GP(4),
        .ADC_BITS    (12),
        .ZERO_CYCLES (16),
        .DATA_CYCLES (48),
        .GROUP_CYCLES(64),
        .N_GROUPS    (16),
        .FRAME_CYCLES(1024),
        .TEST_ITERS  (3)
    ) u_tx (
        .clk      (clk_32),
        .rst_n    (rst_n),
        .sync     (tx_sync),
        .next_amps(tx_next_amps),
        .data     (tx_data)
    );

    assign serial_out    = tx_data;
    assign sync_out      = tx_sync;
    assign next_amps_out = tx_next_amps;

    // -------------------------------------------------------------------------
    // RX — deserializer
    // serial_in comes back from J39 Pin 6 (jumpered to Pin 5 for loopback).
    // sync and next_amps are shared from TX — in a real system these would
    // come over dedicated lines from the ADC front-end.
    // -------------------------------------------------------------------------
    wire [11:0] amp_out [0:63]; // deserialized samples — not driven to pins
    wire        frame_done;

    adc_rx #(
        .N_AMPS      (64),
        .N_ADC_PER_GP(4),
        .ADC_BITS    (12),
        .ZERO_CYCLES (16),
        .GROUP_CYCLES(64),
        .N_GROUPS    (16)
    ) u_rx (
        .clk      (clk_32),
        .rst_n    (rst_n),
        .sync     (tx_sync),
        .next_amps(tx_next_amps),
        .data     (serial_in),   // loopback data from J39 Pin 6
        .amp_out  (amp_out),
        .frame_done(frame_done)
    );

    // -------------------------------------------------------------------------
    // LED0 — frame blink (~1.9 Hz)
    // frame_done fires at 32 MHz / 1024 cycles = 31.25 kHz.
    // Count frames and expose bit [14] -> toggles at 31.25k / 2^15 ≈ 0.95 Hz.
    // -------------------------------------------------------------------------
    reg [14:0] frame_cnt = 15'd0;
    always @(posedge clk_32 or negedge rst_n) begin
        if (!rst_n)      frame_cnt <= 15'd0;
        else if (frame_done) frame_cnt <= frame_cnt + 1'd1;
    end
    assign led0 = ~frame_cnt[14];  // active-low

    // -------------------------------------------------------------------------
    // LED1 — error watchdog
    // If frame_done does not arrive within ~2× frame period the latch sets.
    // 32 MHz * 2048 cycles ≈ 64 µs timeout (well under 2× 31 µs frame).
    // Use a wider timeout so board noise doesn't false-trigger: ~65 ms.
    // 32 MHz * 2^21 ≈ 65.5 ms > 2× frame period of 32 µs.
    // -------------------------------------------------------------------------
    reg [20:0] watchdog = 21'd0;
    reg        err_latch = 1'b0;
    always @(posedge clk_32 or negedge rst_n) begin
        if (!rst_n) begin
            watchdog  <= 21'd0;
            err_latch <= 1'b0;
        end else if (frame_done) begin
            watchdog  <= 21'd0;
        end else begin
            if (watchdog == 21'h1FFFFF)
                err_latch <= 1'b1;
            else
                watchdog <= watchdog + 1'd1;
        end
    end
    assign led1 = ~err_latch;  // active-low: illuminates on error

    // -------------------------------------------------------------------------
    // LED2 — TX heartbeat (toggles on every sync rising edge = every frame)
    // -------------------------------------------------------------------------
    reg sync_d = 1'b0;
    reg heartbeat = 1'b0;
    always @(posedge clk_32 or negedge rst_n) begin
        if (!rst_n) begin
            sync_d    <= 1'b0;
            heartbeat <= 1'b0;
        end else begin
            sync_d <= tx_sync;
            if (tx_sync && !sync_d) heartbeat <= ~heartbeat;
        end
    end
    assign led2 = ~heartbeat;

    // LED3 spare — off
    assign led3 = 1'b1;

endmodule
