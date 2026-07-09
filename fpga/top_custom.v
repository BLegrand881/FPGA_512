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
    output wire        serial_clk,   // 32 MHz bit clock reference for receiver

    // FT600 USB FIFO (U1) — 16-bit synchronous FIFO, 100 MHz
    input  wire        ft600_clk,    // G1  — 100 MHz clock from FT600
    input  wire        ft600_txe_n,  // C1  — TX buffer not full (low = can write)
    input  wire        ft600_rxf_n,  // C2  — RX data available (unused)
    output wire        ft600_wr_n,   // C3  — write strobe (active low)
    output wire        ft600_rd_n,   // B1  — read strobe (active low)
    output wire        ft600_oe_n,   // B2  — output enable (active low)
    output wire        ft600_be0,    // E3  — byte enable 0
    output wire        ft600_be1,    // D3  — byte enable 1
    inout  wire [15:0] ft600_d,      // FT600 16-bit bidirectional data bus

    // Status LEDs (common-cathode to GND, anode via series resistor from FPGA)
    output wire        led_power,    // P16 → D1: solid on when PLL locked
    output wire        led_data      // M13 → D2: blinks when data is flowing
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
    //    cb_clkh    : PLL 32 MHz → ADC board — gated LOW until PLL locks.
    //                 Prevents the ADC board from receiving a glitchy/wrong-
    //                 frequency clock during the ~100 µs PLL acquisition time.
    //    serial_clk : cb_clk32mhz passthrough → J6 Pin 7 (bit-clock reference)
    // -------------------------------------------------------------------------
    assign cb_clkh    = pll_locked ? clk32M : 1'b0;
    assign serial_clk = cb_clk32mhz;

    // -------------------------------------------------------------------------
    // 3. Stimulation / spare outputs — held low
    // -------------------------------------------------------------------------
    assign cb_stim_en    = 1'b0;
    assign cb_stim_clk   = 1'b0;
    assign cb_stim_start = 1'b0;
    assign cb_stim_chb   = 1'b0;

    // -------------------------------------------------------------------------
    // 4. FT600 USB FIFO — ADC data bridge
    //    Packs 12-bit ADC words into 16-bit transfers via async FIFO
    //    (32 MHz write → 100 MHz read), then streams to FT600.
    //    Word format: [3:0]=lane, [15:4]=ADC sample (12-bit)
    // -------------------------------------------------------------------------

    // 4a. Reset synchronizer in ft600_clk domain
    reg [3:0] ft_rst_pipe = 4'hF;
    always @(posedge ft600_clk) begin
        ft_rst_pipe <= {ft_rst_pipe[2:0], 1'b0};
    end
    wire ft_rst_n = ~ft_rst_pipe[3];

    // 4b. Write side (32 MHz): counter test pattern into FIFO
    //     Writes incrementing 16-bit counter at 32 MHz through the async
    //     FIFO to the FT600 at 100 MHz.  Verifies the full CDC path.
    wire        fifo_full;
    reg [15:0]  test_counter = 16'd0;
    wire        fifo_wen = ~fifo_full;

    // Use PLL 32 MHz (clk32M) so this test works without ADC board
    reg [3:0] pll_rst_pipe = 4'hF;
    always @(posedge clk32M) begin
        pll_rst_pipe <= {pll_rst_pipe[2:0], ~pll_locked};
    end
    wire pll_rst_n = ~pll_rst_pipe[3];

    always @(posedge clk32M) begin
        if (!pll_rst_n)
            test_counter <= 16'd0;
        else if (fifo_wen)
            test_counter <= test_counter + 1'b1;
    end

    wire [15:0] fifo_wdata = test_counter;

    // 4c. Async FIFO: 32 MHz → 100 MHz, 16-bit, 1024 deep
    wire        fifo_ren;
    wire [15:0] fifo_rdata;
    wire [15:0] fifo_rdata_next;
    wire        fifo_empty;
    wire        fifo_almost_empty;

    async_fifo #(.W(16), .D(1024)) u_adc_fifo (
        .wclk         (clk32M),
        .wrst_n       (pll_rst_n),
        .wen          (fifo_wen),
        .wdata        (fifo_wdata),
        .full         (fifo_full),
        .rclk         (ft600_clk),
        .rrst_n       (ft_rst_n),
        .ren          (fifo_ren),
        .rdata        (fifo_rdata),
        .rdata_next   (fifo_rdata_next),
        .empty        (fifo_empty),
        .almost_empty (fifo_almost_empty)
    );

    // 4d. FT600 writer (100 MHz): drain FIFO into FT600
    wire [15:0] ft_data_out;
    wire        ft_data_oe;
    wire [1:0]  ft_be;

    ft600_writer u_ft600 (
        .clk              (ft600_clk),
        .rst_n            (ft_rst_n),
        .txe_n            (ft600_txe_n),
        .wr_n             (ft600_wr_n),
        .rd_n             (ft600_rd_n),
        .oe_n             (ft600_oe_n),
        .be               (ft_be),
        .data_out         (ft_data_out),
        .data_oe          (ft_data_oe),
        .fifo_ren         (fifo_ren),
        .fifo_rdata       (fifo_rdata),
        .fifo_rdata_next  (fifo_rdata_next),
        .fifo_empty       (fifo_empty),
        .fifo_almost_empty(fifo_almost_empty)
    );

    assign ft600_be0 = ft_be[0];
    assign ft600_be1 = ft_be[1];
    assign ft600_d   = ft_data_oe ? ft_data_out : 16'bz;

    // -------------------------------------------------------------------------
    // 5. Status LEDs
    //    led_power : on whenever PLL is locked (solid = FPGA running)
    //    led_data  : stretches any out_valid pulse to ~0.1 s so it's visible.
    //                Driven in the cb_clk32mhz domain (ADC clock).
    //                Stretch counter: 32 MHz × 2^22 ≈ 131 ms per blink.
    // -------------------------------------------------------------------------
    assign led_power = pll_locked;

    reg [21:0] data_stretch = 22'd0;
    always @(posedge cb_clk32mhz) begin
        if (|out_valid)
            data_stretch <= 22'h3FFFFF;   // reload to full count on any valid word
        else if (data_stretch != 0)
            data_stretch <= data_stretch - 1'b1;
    end
    assign led_data = (data_stretch != 0);

    // -------------------------------------------------------------------------
    // 6. Unused input anchor — keeps IO buffers in netlist for PULLMODE in LPF
    // -------------------------------------------------------------------------
    wire _unused_ok = &{1'b0,
        cb_chip_reset,
        cb_ac_in,
        cb_imp_test,
        cb_fe_reset,
        cb_spi_clk,
        cb_spi_latch,
        cb_spi_dinr,
        cb_spi_dinl,
        ft600_rxf_n
    };

    // -------------------------------------------------------------------------
    // 7. Reset — synchronous de-assertion in cb_clk32mhz domain
    //    rst_pipe shifts in 0s only after pll_locked is asserted, so the
    //    decoder stays in reset until:
    //      (a) PLL has locked (cb_clkh is stable 32 MHz to ADC board), AND
    //      (b) cb_clk32mhz has been toggling for at least 4 cycles
    //          (confirming the ADC board is responding to our clock).
    // -------------------------------------------------------------------------
    reg [3:0] rst_pipe = 4'hF;
    always @(posedge cb_clk32mhz) begin
        rst_pipe <= {rst_pipe[2:0], ~pll_locked};
    end
    wire rst_n = ~rst_pipe[3];

    // -------------------------------------------------------------------------
    // 8. RX — 8-channel ADC deserializer → 8:4 TDM mux
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
    // 9. Lane framers — 4 independent MSB-first serial output streams
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
