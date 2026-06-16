// =============================================================================
// top.v  —  ECP5-5G Eval Board, 8-channel external ADC receiver
//
// Physical setup:
//   Move JP11 from pins 1-2 to pins 2-3 → VCCIO7 = 1.5 V (J32/J33 bank).
//   Verify LED D23 (+1.5 V indicator) is lit.
//   Connect external ADC device to J32 and J33.
//   Connect logic analyzer to J40 (Bank 0, 3.3 V, no jumper change needed).
//
// Clock domains:
//   clk32M        — FPGA PLL output (12→32 MHz). Drives LED/watchdog logic
//                   and is sent OUT to the device as CLKH (J32 Pin 22 / E4).
//   j33_clk32mhz  — 32 MHz clock received BACK from the device (J33 Pin 14 / D2).
//                   Used to clock the RX deserializer so data is decoded
//                   synchronously to the device's own transmit clock.
//
// J32 (Bank 7, 1.5 V):
//   Pins 5,9,13,17,25,29,33,37 — D[8:1]  8 serial data streams (one per channel)
//   Pin 21 (D5)  CHIP_RESET  — spare input (pulled up)
//   Pin 22 (E4)  CLKH        — 32 MHz clock OUTPUT to device
//
// J33 (Bank 7, 1.5 V):
//   Pin 14 (D2)  CLK32MHZ    — 32 MHz clock INPUT from device → RX decode clock
//   Pin 18 (H4)  READ        — group-boundary pulse (next_amps), broadcast to all 8 ch
//   Pin 22 (H5)  SYNC        — frame sync, broadcast to all 8 channels
//   Pins 25,29 (F3,E2) STIM_EN, STIM_START — outputs to device (held low)
//   Pins 26,30 (G3,F2) STIM_CHB, STIM_CLK  — outputs to device (held low)
//
// J40 (Bank 0, 3.3 V) — logic analyzer output:
//   Pin 1  (K2)  la_clk     — 32 MHz decode clock (passthrough of j33_clk32mhz)
//   Pin 2        GND         — LA ground clip
//   Pin 3  (A15) la_data[0] — out_valid[0]: channel pair 0/1 active
//   Pin 4  (F1)  la_data[1] — out_valid[1]: channel pair 2/3 active
//   Pin 5  (H2)  la_data[2] — out_valid[2]: channel pair 4/5 active
//   Pin 6  (G1)  la_data[3] — out_valid[3]: channel pair 6/7 active
//
// LEDs (active-low):
//   LED[0]  A13  — blinks ~1 Hz when output data is flowing (watchdog healthy)
//   LED[1]  A12  — latches ON if no output data for ~65 ms (RX dead)
//   LED[2]  B19  — toggles on every incoming SYNC rising edge
//   LED[3]  A18  — ON when PLL is locked
//   LED[4-7]     — spare (off)
//
// Reset: rstn — SW4 (P4), active-low
// =============================================================================

module top (
    input  wire        clk_x1,         // 12 MHz FTDI clock (ball A10, JP2 installed)
    input  wire        rstn,           // SW4 (ball P4), active-low
    output wire  [7:0] LED,            // LEDs D2-D9 (active-low)

    // J39 Pin 9 (D11) — divided clock for scope verification
    output wire        clk_div_out,

    // J32 — 8 serial data streams + control (Bank 7, 1.5 V)
    input  wire [8:1]  j32_d,          // D[8:1]: one serial bitstream per channel
    input  wire        j32_chip_reset, // spare input
    output wire        j32_clkh,       // 32 MHz clock OUTPUT to device

    // J33 — control signals (Bank 7, 1.5 V)
    input  wire        j33_ac_in,
    input  wire        j33_imp_test,
    input  wire        j33_fe_reset,
    input  wire        j33_spi_clk,
    input  wire        j33_spi_latch,
    input  wire        j33_clk32mhz,   // 32 MHz clock INPUT from device → decode clock
    input  wire        j33_spi_dinr,   // spare input
    input  wire        j33_read,       // group-boundary pulse (next_amps), all channels
    input  wire        j33_spi_dinl,   // spare input
    input  wire        j33_sync,       // frame sync, broadcast to all channels
    output wire        j33_stim_en,
    output wire        j33_stim_chb,
    output wire        j33_stim_start,
    output wire        j33_stim_clk,

    // J40 — logic analyzer output (Bank 0, 3.3 V)
    output wire        la_clk,         // J40 Pin 1 (K2)  — decode clock for LA
    output wire [3:0]  la_data         // J40 Pins 3-6    — out_valid[3:0]
);

    wire rst;
    wire clk32M;
    wire pll_locked;

    assign rst = ~rstn;

    // -------------------------------------------------------------------------
    // 1. PLL — 12 MHz -> 32 MHz (drives LED/watchdog domain and CLKH output)
    // -------------------------------------------------------------------------
    pll_32mhz pll_inst (
        .clk_in (clk_x1),
        .reset  (rst),
        .clk_out(clk32M),
        .locked (pll_locked)
    );

    // -------------------------------------------------------------------------
    // 2. Reset — synchronous de-assertion in clk32M domain, held until PLL locks
    // -------------------------------------------------------------------------
    reg [3:0] rst_pipe = 4'hF;
    always @(posedge clk32M) begin
        if (rst || !pll_locked) rst_pipe <= 4'hF;
        else                    rst_pipe <= {rst_pipe[2:0], 1'b0};
    end
    wire rst_n = ~rst_pipe[3];

    // Reset synchronizer for the j33_clk32mhz domain (async assert, sync deassert)
    reg [3:0] rst_rx_pipe = 4'hF;
    always @(posedge j33_clk32mhz or negedge rst_n) begin
        if (!rst_n) rst_rx_pipe <= 4'hF;
        else        rst_rx_pipe <= {rst_rx_pipe[2:0], 1'b0};
    end
    wire rst_rx_n = ~rst_rx_pipe[3];

    // -------------------------------------------------------------------------
    // 3. Clock outputs
    //    CLKH (j32_clkh): FPGA PLL 32 MHz → device (device uses this to TX data)
    //    la_clk          : same device decode clock re-exported for LA reference
    // -------------------------------------------------------------------------
    assign j32_clkh = clk32M;
    assign la_clk   = j33_clk32mhz;

    // -------------------------------------------------------------------------
    // 4. Stimulation outputs — held low until needed
    // -------------------------------------------------------------------------
    assign j33_stim_en    = 1'b0;
    assign j33_stim_chb   = 1'b0;
    assign j33_stim_start = 1'b0;
    assign j33_stim_clk   = 1'b0;

    // -------------------------------------------------------------------------
    // Unused input anchor — keeps IO buffers in netlist so LPF PULLMODE applies
    // -------------------------------------------------------------------------
    wire _unused_ok = &{1'b0,
        j32_chip_reset,
        j33_ac_in,
        j33_imp_test,
        j33_fe_reset,
        j33_spi_clk,
        j33_spi_latch,
        j33_spi_dinr,
        j33_spi_dinl
    };

    // -------------------------------------------------------------------------
    // 5. RX — 8-channel deserializer + 8:4 TDM mux
    //
    //   Clock  : j33_clk32mhz  — device's own transmit clock (decode domain)
    //   Data   : j32_d[8:1]    — 8 independent serial bitstreams, one per channel
    //   Sync   : j33_sync      — frame boundary, broadcast to all 8 channels
    //   Next   : j33_read      — group-boundary (next_amps), broadcast to all 8 ch
    //
    //   Output : out_valid[3:0] — one strobe per output lane (pair of channels)
    //            out_word[0:3]  — 12-bit ADC word per lane when valid
    //            out_chsel[3:0] — which sub-channel (0=even, 1=odd) is on each lane
    // -------------------------------------------------------------------------
    wire [3:0]  out_valid;
    wire [11:0] out_word [0:3];
    wire [3:0]  out_chsel;

    rx_process_mux #(
        .N_CH         (8),
        .N_ADC_PER_GP (4),
        .ADC_BITS     (12),
        .ZERO_CYCLES  (16),
        .GROUP_CYCLES (64),
        .N_GROUPS     (16),
        .SAMPS_PER_FR (64),
        .RX_FIFO_DEPTH(16),
        .TX_FIFO_DEPTH(16)
    ) u_mux (
        .clk          (j33_clk32mhz),
        .rst_n        (rst_rx_n),
        .data_in      (j32_d[8:1]),           // D[8] → ch7 … D[1] → ch0
        .sync_in      ({8{j33_sync}}),         // shared frame sync → all channels
        .next_amps_in ({8{j33_read}}),         // shared group boundary → all channels
        .out_valid    (out_valid),
        .out_word     (out_word),
        .out_chsel    (out_chsel)
    );

    // -------------------------------------------------------------------------
    // 6. Logic analyzer outputs on J40
    //    la_data[3:0] = out_valid[3:0]: shows which output lane is active each cycle
    //    la_clk = j33_clk32mhz (assigned above): LA samples on this reference
    // -------------------------------------------------------------------------
    assign la_data = out_valid;

    // -------------------------------------------------------------------------
    // 7. Divided clock output on J39 Pin 9 (D11) — scope frequency reference
    // -------------------------------------------------------------------------
    reg [9:0] clk_div_cnt = 10'd0;
    always @(posedge clk32M) clk_div_cnt <= clk_div_cnt + 1'd1;
    assign clk_div_out = clk_div_cnt[9];

    // -------------------------------------------------------------------------
    // 8. LED status (clk32M domain — CDC-safe: out_valid double-flopped)
    // -------------------------------------------------------------------------

    // Two-stage synchronizer: bring out_valid into clk32M domain for LED/watchdog
    reg [3:0] ov_s1 = 4'd0, ov_s2 = 4'd0;
    always @(posedge clk32M) begin
        ov_s1 <= out_valid;
        ov_s2 <= ov_s1;
    end
    wire any_valid = |ov_s2;

    // LED[0] — blinks ~1 Hz while output data is flowing
    reg [19:0] valid_cnt = 20'd0;
    always @(posedge clk32M or negedge rst_n) begin
        if (!rst_n)       valid_cnt <= 20'd0;
        else if (any_valid) valid_cnt <= valid_cnt + 1'd1;
    end

    // LED[1] — watchdog: latches if no out_valid for ~65 ms
    reg [20:0] watchdog = 21'd0;
    reg        wdog_err = 1'b0;
    always @(posedge clk32M or negedge rst_n) begin
        if (!rst_n) begin
            watchdog <= 21'd0;
            wdog_err <= 1'b0;
        end else if (any_valid) begin
            watchdog <= 21'd0;
        end else begin
            if (watchdog == 21'h1FFFFF) wdog_err <= 1'b1;
            else                        watchdog <= watchdog + 1'd1;
        end
    end

    // LED[2] — toggles on every rising edge of incoming SYNC
    reg sync_d    = 1'b0;
    reg heartbeat = 1'b0;
    always @(posedge clk32M or negedge rst_n) begin
        if (!rst_n) begin
            sync_d    <= 1'b0;
            heartbeat <= 1'b0;
        end else begin
            sync_d <= j33_sync;
            if (j33_sync && !sync_d) heartbeat <= ~heartbeat;
        end
    end

    // LED assignments (active-low)
    assign LED[0] = ~valid_cnt[19];   // blinks ~1 Hz when data flows
    assign LED[1] = ~wdog_err;        // ON if no data for ~65 ms
    assign LED[2] = ~heartbeat;       // toggles on each frame sync
    assign LED[3] = ~pll_locked;      // ON when PLL locked
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
