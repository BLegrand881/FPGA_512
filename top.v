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
//   LED[0]  A13  — blinks ~1 Hz when frames complete (RX healthy)
//   LED[1]  A12  — latches ON on watchdog timeout (no frame_done)
//   LED[2]  B19  — TX heartbeat, toggles every frame sync
//   LED[3]  A18  — ON when PLL is locked
//   LED[4]  B18  — spare (off)
//   LED[5]  C17  — spare (off)
//   LED[6]  A17  — spare (off)
//   LED[7]  B17  — spare (off)
//
// Reset: rstn — SW4 (P4), active-low
// =============================================================================

module top (
    input  wire        clk_x1,         // 12 MHz FTDI clock (ball A10, JP2 installed)
    input  wire        rstn,           // SW4 (ball P4), active-low
    output wire  [7:0] LED,            // LEDs D2-D9 (active-low)

    // J39 expansion header
    output wire        serial_out,     // J39 Pin 5 (B15) — TX bitstream
    input  wire        serial_in,      // J39 Pin 6 (C15) — RX bitstream (jumper from Pin 5)
    output wire        sync_out,       // J39 Pin 7 (B13) — frame sync
    output wire        next_amps_out,  // J39 Pin 8 (B20) — group boundary
    output wire        clk_div_out     // J39 Pin 9 (D11) — 32 MHz / 1024 ≈ 31 kHz, scope-measurable
);

    wire rst;
    wire clk32M;
    wire pll_locked;

    assign rst = ~rstn;  // active-high internal reset

    // -------------------------------------------------------------------------
    // 1. PLL — 12 MHz -> 32 MHz
    //    CLKOP_DIV=15 puts VCO at 480 MHz (required range: 400-800 MHz).
    //    Output = 480 / 15 = 32 MHz.
    // -------------------------------------------------------------------------
    pll_32mhz pll_inst (
        .clk_in (clk_x1),
        .reset  (rst),
        .clk_out(clk32M),
        .locked (pll_locked)
    );

    // -------------------------------------------------------------------------
    // 2. Reset — synchronous de-assertion, held until PLL locks
    // -------------------------------------------------------------------------
    reg [3:0] rst_pipe = 4'hF;
    always @(posedge clk32M) begin
        if (rst || !pll_locked)
            rst_pipe <= 4'hF;
        else
            rst_pipe <= {rst_pipe[2:0], 1'b0};
    end
    wire rst_n = ~rst_pipe[3];  // active-low, clean synchronous release

    // -------------------------------------------------------------------------
    // 3. TX — serial stream generator
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
        .clk      (clk32M),
        .rst_n    (rst_n),
        .sync     (tx_sync),
        .next_amps(tx_next_amps),
        .data     (tx_data)
    );

    assign serial_out    = tx_data;
    assign sync_out      = tx_sync;
    assign next_amps_out = tx_next_amps;

    // -------------------------------------------------------------------------
    // 4. RX — deserializer (loopback: reads serial_in, jumpered to serial_out)
    // sync/next_amps shared from TX — in a real system these come from the ADC.
    // -------------------------------------------------------------------------
    wire [64*12-1:0] amp_out;   // flat: sample i = amp_out[i*12 +: 12]
    wire             frame_done;

    adc_rx #(
        .N_AMPS      (64),
        .N_ADC_PER_GP(4),
        .ADC_BITS    (12),
        .ZERO_CYCLES (16),
        .GROUP_CYCLES(64),
        .N_GROUPS    (16)
    ) u_rx (
        .clk       (clk32M),
        .rst_n     (rst_n),
        .sync      (tx_sync),
        .next_amps (tx_next_amps),
        .data      (serial_in),
        .amp_out   (amp_out),
        .frame_done(frame_done)
    );

    // -------------------------------------------------------------------------
    // 5. Divided clock output — scope/LA verification of PLL frequency
    //    32 MHz / 2^10 = 31.25 kHz on J39 Pin 9 (D11)
    // -------------------------------------------------------------------------
    reg [9:0] clk_div_cnt = 10'd0;
    always @(posedge clk32M) clk_div_cnt <= clk_div_cnt + 1'd1;
    assign clk_div_out = clk_div_cnt[9];

    // -------------------------------------------------------------------------
    // 6. LED status
    // -------------------------------------------------------------------------

    // LED[0] — blinks ~1 Hz on healthy frame_done pulses
    // frame_done fires at 32 MHz / 1024 = 31.25 kHz
    // bit[14] toggles at 31.25 kHz / 2^15 ≈ 0.95 Hz
    reg [14:0] frame_cnt = 15'd0;
    always @(posedge clk32M or negedge rst_n) begin
        if (!rst_n)          frame_cnt <= 15'd0;
        else if (frame_done) frame_cnt <= frame_cnt + 1'd1;
    end

    // LED[1] — combined error latch:
    //   (a) watchdog: frame_done stopped arriving (RX dead)
    //   (b) data check: amp_out[0] differs between repeat cycles (bit error)
    //
    // Watchdog: timeout ~65 ms (32 MHz * 2^21 >> 1 frame = 32 µs)
    reg [20:0] watchdog  = 21'd0;
    reg        wdog_err  = 1'b0;
    always @(posedge clk32M or negedge rst_n) begin
        if (!rst_n) begin
            watchdog <= 21'd0;
            wdog_err <= 1'b0;
        end else if (frame_done) begin
            watchdog <= 21'd0;
        end else begin
            if (watchdog == 21'h1FFFFF)
                wdog_err <= 1'b1;
            else
                watchdog <= watchdog + 1'd1;
        end
    end

    // Data checker: TX cycles TEST_ITERS=3 patterns then repeats.
    // amp_out[0] at frame 0 == amp_out[0] at frame 3 == frame 6 ...
    // Latch amp_out[0] on the first frame-0, compare on every subsequent one.
    reg [1:0]  frame_mod  = 2'd0;
    reg [11:0] ref_amp0   = 12'd0;
    reg        ref_valid  = 1'b0;
    reg        data_err   = 1'b0;
    always @(posedge clk32M or negedge rst_n) begin
        if (!rst_n) begin
            frame_mod <= 2'd0;
            ref_amp0  <= 12'd0;
            ref_valid <= 1'b0;
            data_err  <= 1'b0;
        end else if (frame_done) begin
            if (frame_mod == 2'd0) begin
                if (!ref_valid) begin
                    ref_amp0  <= amp_out[0 +: 12];   // latch sample[0] on first frame-0
                    ref_valid <= 1'b1;
                end else if (amp_out[0 +: 12] != ref_amp0) begin
                    data_err <= 1'b1;          // mismatch on repeat — bit error
                end
            end
            frame_mod <= (frame_mod == 2'd2) ? 2'd0 : frame_mod + 1'd1;
        end
    end

    // LED[2] — TX heartbeat: toggles on every sync rising edge
    reg sync_d    = 1'b0;
    reg heartbeat = 1'b0;
    always @(posedge clk32M or negedge rst_n) begin
        if (!rst_n) begin
            sync_d    <= 1'b0;
            heartbeat <= 1'b0;
        end else begin
            sync_d <= tx_sync;
            if (tx_sync && !sync_d) heartbeat <= ~heartbeat;
        end
    end

    // LED assignments (active-low: 0 = illuminated, 1 = off)
    assign LED[0] = ~frame_cnt[14];          // blinks ~1 Hz when frames arrive
    assign LED[1] = ~(wdog_err | data_err);  // illuminates on any error
    assign LED[2] = ~heartbeat;              // TX heartbeat toggle
    assign LED[3] = ~pll_locked;             // illuminates when PLL locked
    assign LED[4] = 1'b1;
    assign LED[5] = 1'b1;
    assign LED[6] = 1'b1;
    assign LED[7] = 1'b1;

endmodule


// =============================================================================
// PLL: 12 MHz -> 32 MHz using ECP5 EHXPLLL primitive
//
// CLKI_DIV=3, CLKFB_DIV=8, CLKOP_DIV=15
//   PFD  = 12 / 3         =  4 MHz
//   VCO  =  4 * 8 * 15    = 480 MHz  (in-range: 400-800 MHz)
//   CLKOP = 480 / 15      = 32 MHz
// =============================================================================
module pll_32mhz (
    input  wire clk_in,
    input  wire reset,
    output wire clk_out,
    output wire locked
);
    wire vcc = 1'b1;
    wire gnd = 1'b0;
    wire clk_fb;

    EHXPLLL #(
        .CLKI_DIV        (3),
        .CLKFB_DIV       (8),
        .CLKOP_DIV       (15),          // VCO = 32 * 15 = 480 MHz (in range)
        .CLKOP_ENABLE    ("ENABLED"),
        .CLKOS_ENABLE    ("DISABLED"),
        .CLKOS2_ENABLE   ("DISABLED"),
        .CLKOS3_ENABLE   ("DISABLED"),
        .OUTDIVIDER_MUXA ("DIVA"),
        .FEEDBK_PATH     ("CLKOP"),
        .PLLRST_ENA      ("ENABLED")
    ) pll_macro (
        .CLKI        (clk_in),
        .CLKFB       (clk_fb),
        .PHASESEL1   (gnd), .PHASESEL0(gnd),
        .PHASEDIR    (gnd), .PHASESTEP(gnd),
        .PHASELOADREG(gnd), .STDBY    (gnd),
        .RST         (reset),
        .ENCLKOP     (vcc), .ENCLKOS(gnd), .ENCLKOS2(gnd), .ENCLKOS3(gnd),
        .CLKOP       (clk_fb),
        .LOCK        (locked)
    );

    assign clk_out = clk_fb;
endmodule
