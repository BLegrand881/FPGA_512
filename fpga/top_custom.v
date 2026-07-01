// =============================================================================
// top_custom.v  —  Custom FPGA board (LFE5U-25F-7BG256I, BGA256)
//
// Receives 8 ADC serial streams from the UWB ADC board via CN1,
// decodes them, and outputs 4 framed serial lanes + 32 MHz clock on J6.
//
// Pin assignments (from KiCad netlist "FPGA board.net"):
// -------------------------------------------------------
//   clk_16m      A7    — 16 MHz crystal X1
//
//   cb_d[1]      B15   — ADC board data stream 1 (CB_D<1>)
//   cb_d[2]      B16   — ADC board data stream 2 (CB_D<2>)
//   cb_d[3]      C15   — ADC board data stream 3 (CB_D<3>)
//   cb_d[4]      C16   — ADC board data stream 4 (CB_D<4>)
//   cb_d[5]      J15   — ADC board data stream 5 (CB_D<5>)
//   cb_d[6]      K15   — ADC board data stream 6 (CB_D<6>)
//   cb_d[7]      K14   — ADC board data stream 7 (CB_D<7>)
//   cb_d[8]      J14   — ADC board data stream 8 (CB_D<8>)
//   cb_clk32mhz  K16   — 32 MHz clock IN  from ADC board (source-sync decode clock)
//   cb_read      J13   — next_amps group boundary pulse
//   cb_sync      H14   — frame sync (currently unused by decoder)
//   cb_clkh      J16   — 32 MHz clock OUT to ADC board (drives ADC TX)
//
//   serial_out[0] A15  — J6 Pin 3: framed serial lane 0 (SYNC 0xA35)
//   serial_out[1] A14  — J6 Pin 4: framed serial lane 1 (SYNC 0xB46)
//   serial_out[2] B14  — J6 Pin 5: framed serial lane 2 (SYNC 0xC57)
//   serial_out[3] A13  — J6 Pin 6: framed serial lane 3 (SYNC 0xD68)
//   serial_clk    A12  — J6 Pin 7: 32 MHz bit clock (cb_clk32mhz passthrough)
//
// Clock domains:
//   clk32M       — PLL output 16→32 MHz; drives cb_clkh (ADC board TX clock)
//   cb_clk32mhz  — 32 MHz received back from ADC board; decodes ADC serial data
//                  and clocks all lane_framer serial outputs (source-synchronous)
//
// Build:
//   make synth-custom   (Yosys + nextpnr-ecp5 --25k --package CABGA256)
//   make prog-custom    (openFPGALoader with J2 JTAG)
// =============================================================================

module top (
    input  wire        clk_16m,      // 16 MHz crystal (A7)

    // CN1 — ADC board data interface
    input  wire [8:1]  cb_d,         // 8 serial bitstreams from ADC board
    input  wire        cb_clk32mhz,  // 32 MHz clock from ADC board → decode clock
    input  wire        cb_read,      // next_amps: group-boundary pulse
    input  wire        cb_sync,      // frame sync (passed to decoder, currently unused)
    output wire        cb_clkh,      // 32 MHz clock to ADC board

    // CN1 — spare/stimulation outputs (held inactive)
    output wire        cb_stim_en,
    output wire        cb_stim_clk,
    output wire        cb_stim_start,
    output wire        cb_stim_chb,

    // CN1 — spare inputs (anchored to prevent LPF errors)
    input  wire        cb_chip_reset,
    input  wire        cb_ac_in,
    input  wire        cb_imp_test,
    input  wire        cb_fe_reset,
    input  wire        cb_spi_clk,
    input  wire        cb_spi_latch,
    input  wire        cb_spi_dinr,
    input  wire        cb_spi_dinl,

    // J6 — framed serial output (4 lanes + bit clock)
    output wire [3:0]  serial_out,   // lanes 0–3, MSB-first, 1 bit per clock
    output wire        serial_clk    // 32 MHz bit clock reference for receiver
);

    // -------------------------------------------------------------------------
    // 1. PLL — 16 MHz → 32 MHz
    //    Drives cb_clkh (output to ADC board so it can transmit).
    //    VCO = 16 * 2 * 15 = 480 MHz (within ECP5 400–800 MHz range)
    //    CLKOP = 480 / 15 = 32 MHz
    // -------------------------------------------------------------------------
    wire clk32M;
    wire pll_locked;

    pll_16to32 pll_inst (
        .clk_in (clk_16m),
        .clk_out(clk32M),
        .locked (pll_locked)
    );

    // -------------------------------------------------------------------------
    // 2. Clock outputs
    //    cb_clkh    : PLL 32 MHz → ADC board (ADC board uses this to launch data)
    //    serial_clk : cb_clk32mhz passthrough → J6 Pin 7 (bit-clock reference)
    // -------------------------------------------------------------------------
    assign cb_clkh    = clk32M;
    assign serial_clk = cb_clk32mhz;

    // -------------------------------------------------------------------------
    // 3. Stimulation / spare outputs — held low
    // -------------------------------------------------------------------------
    assign cb_stim_en    = 1'b0;
    assign cb_stim_clk   = 1'b0;
    assign cb_stim_start = 1'b0;
    assign cb_stim_chb   = 1'b0;

    // -------------------------------------------------------------------------
    // 4. Unused input anchor — keeps IO buffers in netlist for PULLMODE in LPF
    // -------------------------------------------------------------------------
    wire _unused_ok = &{1'b0,
        pll_locked,
        cb_chip_reset,
        cb_ac_in,
        cb_imp_test,
        cb_fe_reset,
        cb_spi_clk,
        cb_spi_latch,
        cb_spi_dinr,
        cb_spi_dinl
    };

    // -------------------------------------------------------------------------
    // 5. Reset — synchronous de-assertion in cb_clk32mhz domain
    //    Held asserted until cb_clk32mhz is toggling (2 cycles minimum).
    //    No PLL lock dependency since cb_clk32mhz comes from the ADC board.
    // -------------------------------------------------------------------------
    reg [3:0] rst_pipe = 4'hF;
    always @(posedge cb_clk32mhz) begin
        rst_pipe <= {rst_pipe[2:0], 1'b0};
    end
    wire rst_n = ~rst_pipe[3];

    // -------------------------------------------------------------------------
    // 6. RX — 8-channel ADC deserializer → 8:4 TDM mux
    //
    //    cb_d[k] maps directly to decoder channel k-1 (data_in[k-1]).
    //    cb_read  → next_amps (group-boundary reset, broadcast to all 8 ch)
    //    cb_sync  → sync_in   (frame sync; unused internally, kept for IO buf)
    //    Clock    → cb_clk32mhz (source-synchronous with ADC board output)
    // -------------------------------------------------------------------------
    wire [3:0]  out_valid;
    wire [47:0] out_word;
    wire [3:0]  out_chsel;

    rx_process_mux #(
        .N_CH         (8),
        .N_ADC_PER_GP (4),
        .ADC_BITS     (12),
        .ZERO_CYCLES  (11),
        .GROUP_CYCLES (64),
        .N_GROUPS     (16),
        .SAMPS_PER_FR (64),
        .RX_FIFO_DEPTH(16),
        .TX_FIFO_DEPTH(16)
    ) u_mux (
        .clk          (cb_clk32mhz),
        .rst_n        (rst_n),
        .data_in      (cb_d[8:1]),
        .sync_in      ({8{cb_sync}}),
        .next_amps_in ({8{cb_read}}),
        .out_valid    (out_valid),
        .out_word     (out_word),
        .out_chsel    (out_chsel)
    );

    wire _chsel_unused = &{1'b0, out_chsel};

    // -------------------------------------------------------------------------
    // 7. Lane framers — 4 independent MSB-first serial output streams
    //
    //    Frame format per lane (132 words × 12 bits = 1584 bit-clock cycles):
    //      [SYNC(12b)][LANE_ID(12b)][CYCLE_CNT(12b)][DATA×128][CRC-12(12b)]
    //    Sync words: 0xA35 / 0xB46 / 0xC57 / 0xD68  (DC-balanced, unique per lane)
    //    Receiver samples serial_out on FALLING edge of serial_clk.
    // -------------------------------------------------------------------------
    lane_framer #(.SYNC_WORD(12'hA35), .LANE_NUM(2'd0)) u_frame0 (
        .clk       (cb_clk32mhz),
        .rst_n     (rst_n),
        .in_valid  (out_valid[0]),
        .in_word   (out_word[0*12 +: 12]),
        .serial_out(serial_out[0])
    );

    lane_framer #(.SYNC_WORD(12'hB46), .LANE_NUM(2'd1)) u_frame1 (
        .clk       (cb_clk32mhz),
        .rst_n     (rst_n),
        .in_valid  (out_valid[1]),
        .in_word   (out_word[1*12 +: 12]),
        .serial_out(serial_out[1])
    );

    lane_framer #(.SYNC_WORD(12'hC57), .LANE_NUM(2'd2)) u_frame2 (
        .clk       (cb_clk32mhz),
        .rst_n     (rst_n),
        .in_valid  (out_valid[2]),
        .in_word   (out_word[2*12 +: 12]),
        .serial_out(serial_out[2])
    );

    lane_framer #(.SYNC_WORD(12'hD68), .LANE_NUM(2'd3)) u_frame3 (
        .clk       (cb_clk32mhz),
        .rst_n     (rst_n),
        .in_valid  (out_valid[3]),
        .in_word   (out_word[3*12 +: 12]),
        .serial_out(serial_out[3])
    );

endmodule


// =============================================================================
// PLL: 16 MHz → 32 MHz using ECP5 EHXPLLL primitive
//
//   CLKI_DIV=1, CLKFB_DIV=2, CLKOP_DIV=15
//   PFD  = 16 / 1         = 16 MHz
//   VCO  = 16 × 2 × 15   = 480 MHz  (in-range: 400–800 MHz)
//   CLKOP = 480 / 15      = 32 MHz
// =============================================================================
module pll_16to32 (
    input  wire clk_in,
    output wire clk_out,
    output wire locked
);
    wire vcc = 1'b1;
    wire gnd = 1'b0;
    wire clk_fb;

    EHXPLLL #(
        .CLKI_DIV        (1),
        .CLKFB_DIV       (2),
        .CLKOP_DIV       (15),
        .CLKOP_ENABLE    ("ENABLED"),
        .CLKOS_ENABLE    ("DISABLED"),
        .CLKOS2_ENABLE   ("DISABLED"),
        .CLKOS3_ENABLE   ("DISABLED"),
        .OUTDIVIDER_MUXA ("DIVA"),
        .FEEDBK_PATH     ("CLKOP"),
        .PLLRST_ENA      ("DISABLED")
    ) pll_macro (
        .CLKI        (clk_in),
        .CLKFB       (clk_fb),
        .PHASESEL1   (gnd), .PHASESEL0(gnd),
        .PHASEDIR    (gnd), .PHASESTEP(gnd),
        .PHASELOADREG(gnd), .STDBY    (gnd),
        .RST         (gnd),
        .ENCLKOP     (vcc), .ENCLKOS(gnd), .ENCLKOS2(gnd), .ENCLKOS3(gnd),
        .CLKOP       (clk_fb),
        .LOCK        (locked)
    );

    assign clk_out = clk_fb;
endmodule
