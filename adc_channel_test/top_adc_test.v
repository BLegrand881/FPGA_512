// =============================================================================
// top_adc_test.v — Single-channel ADC bringup test
//
// Samples one ADC data channel (selectable via CHANNEL_SEL), deserializes it,
// zero-pads to 16 bits, and streams raw samples over USB via FT600.
//
// cb_fe_reset held HIGH → amplifier reset → ADC outputs ~2048 midscale.
// No framing, no CRC — just raw {4'b0, sample[11:0]} over USB.
//
// Channel map:
//   1=B15  2=B16  3=C15  4=C16  5=J15  6=K15  7=K14  8=J14
// =============================================================================

module top (
    input  wire        clk_16m,       // A7  — 16 MHz crystal

    // CN1 — ADC board interface
    input  wire [8:1]  cb_d,          // 8 serial data channels
    input  wire        cb_clk32mhz,   // K16 — 32 MHz clock from ADC board
    input  wire        cb_read,       // J13 — group boundary pulse
    input  wire        cb_sync,       // H14 — frame sync (unused)
    output wire        cb_clkh,       // J16 — clock to ADC board

    // CN1 — control outputs
    output wire        cb_chip_reset, // D14 — boot pulse
    output wire        cb_fe_reset,   // E16 — amplifier reset (held HIGH)
    output wire        cb_stim_en,
    output wire        cb_stim_clk,
    output wire        cb_stim_start,
    output wire        cb_stim_chb,

    // CN1 — spare inputs
    input  wire        cb_ac_in,
    input  wire        cb_imp_test,

    // CN1 — SPI outputs
    output wire        cb_spi_clk,
    output wire        cb_spi_latch,
    output wire        cb_spi_dinr,
    output wire        cb_spi_dinl,

    // Button
    input  wire        btn_trigger,   // N13 — active-low

    // FT600 USB FIFO
    input  wire        ft600_clk,     // G1  — 100 MHz
    input  wire        ft600_txe_n,   // C1
    input  wire        ft600_rxf_n,   // C2
    output wire        ft600_wr_n,    // C3
    output wire        ft600_rd_n,    // B1
    output wire        ft600_oe_n,    // B2
    output wire        ft600_be0,     // E3
    output wire        ft600_be1,     // D3
    inout  wire [15:0] ft600_d,       // 16-bit data bus

    // LEDs
    output wire        led_spi,       // P16 — SPI done
    output wire        led_data,      // M13 — ADC data flowing
    output wire        led_usb        // N14 — USB active
);

    // =====================================================================
    // Channel selection — set dynamically via USB command from host
    // =====================================================================
    // Default channel 5 (J15, known good). Host sends a 16-bit word
    // via FT_WritePipe(0x02) with channel number 1-8 in lower bits.
    // 1=B15  2=B16  3=C15  4=C16  5=J15  6=K15  7=K14  8=J14

    // =====================================================================
    // PLL — 16 → 32 MHz (for pll_locked gating only)
    // =====================================================================
    wire clk32M, pll_locked;
    pll_16to32 pll_inst (
        .clk_in (clk_16m),
        .clk_out(clk32M),
        .locked (pll_locked)
    );

    // Clock to ADC board: 16 MHz (matching working bringup config)
    assign cb_clkh = clk_16m;

    // =====================================================================
    // Control outputs
    // =====================================================================
    assign cb_fe_reset   = 1'b1;  // amplifier reset HIGH → midscale output
    assign cb_stim_en    = 1'b0;
    assign cb_stim_clk   = 1'b0;
    assign cb_stim_start = 1'b0;
    assign cb_stim_chb   = 1'b0;

    // Chip reset: 65 ms high pulse on boot
    reg [20:0] reset_ctr = 21'd0;
    wire reset_done = reset_ctr[20];
    always @(posedge clk_16m)
        if (!reset_done)
            reset_ctr <= reset_ctr + 1'b1;
    assign cb_chip_reset = ~reset_done;

    // =====================================================================
    // SPI trigger
    // =====================================================================
    wire spi_done;
    spi_trigger #(.CLK_DIVIDER(16)) u_spi (
        .clk       (clk_16m),
        .btn_n     (btn_trigger),
        .spi_sig1  (cb_spi_dinl),
        .spi_sig2  (cb_spi_dinr),
        .spi_clk_o (cb_spi_clk),
        .spi_latch (cb_spi_latch),
        .done      (spi_done)
    );

    // =====================================================================
    // Reset synchronizers
    // =====================================================================

    // ADC clock domain reset
    reg [3:0] rst_pipe = 4'hF;
    always @(posedge cb_clk32mhz)
        rst_pipe <= {rst_pipe[2:0], ~pll_locked};
    wire rst_n = ~rst_pipe[3];

    // FT600 clock domain reset
    reg [3:0] ft_rst_pipe = 4'hF;
    always @(posedge ft600_clk)
        ft_rst_pipe <= {ft_rst_pipe[2:0], 1'b0};
    wire ft_rst_n = ~ft_rst_pipe[3];

    // =====================================================================
    // FT600 bidirectional — write ADC data, read channel commands
    // =====================================================================
    wire [15:0] ft_data_out;
    wire        ft_data_oe;
    wire [1:0]  ft_be;
    wire        cmd_valid;
    wire [15:0] cmd_data;

    wire        fifo_ren;
    wire [15:0] fifo_rdata;
    wire [15:0] fifo_rdata_next;
    wire        fifo_empty;
    wire        fifo_almost_empty;

    ft600_writer u_ft600 (
        .clk              (ft600_clk),
        .rst_n            (ft_rst_n),
        .txe_n            (ft600_txe_n),
        .rxf_n            (ft600_rxf_n),
        .wr_n             (ft600_wr_n),
        .rd_n             (ft600_rd_n),
        .oe_n             (ft600_oe_n),
        .be               (ft_be),
        .data_out         (ft_data_out),
        .data_oe          (ft_data_oe),
        .ft_data_in       (ft600_d),
        .cmd_valid        (cmd_valid),
        .cmd_data         (cmd_data),
        .fifo_ren         (fifo_ren),
        .fifo_rdata       (fifo_rdata),
        .fifo_rdata_next  (fifo_rdata_next),
        .fifo_empty       (fifo_empty),
        .fifo_almost_empty(fifo_almost_empty)
    );

    assign ft600_be0 = ft_be[0];
    assign ft600_be1 = ft_be[1];
    assign ft600_d   = ft_data_oe ? ft_data_out : 16'bz;

    // =====================================================================
    // CDC: channel command from ft600_clk → cb_clk32mhz
    // =====================================================================
    reg [3:0] cmd_ch_ft = 4'd5;
    reg       cmd_toggle_ft = 1'b0;

    always @(posedge ft600_clk or negedge ft_rst_n) begin
        if (!ft_rst_n) begin
            cmd_ch_ft     <= 4'd5;
            cmd_toggle_ft <= 1'b0;
        end else if (cmd_valid && cmd_data[3:0] >= 1 && cmd_data[3:0] <= 8) begin
            cmd_ch_ft     <= cmd_data[3:0];
            cmd_toggle_ft <= ~cmd_toggle_ft;
        end
    end

    reg [2:0] toggle_sync = 3'b0;
    always @(posedge cb_clk32mhz)
        toggle_sync <= {toggle_sync[1:0], cmd_toggle_ft};
    wire cmd_valid_adc = (toggle_sync[2] != toggle_sync[1]);

    // Channel select register with brief reset on change
    reg [3:0] channel_sel = 4'd5;
    reg [7:0] ch_reset_ctr = 8'd0;
    wire ch_changing = (ch_reset_ctr != 0);

    always @(posedge cb_clk32mhz) begin
        if (cmd_valid_adc) begin
            channel_sel <= cmd_ch_ft;
            ch_reset_ctr <= 8'd255;  // ~8 us reset for deserializer re-sync
        end else if (ch_reset_ctr != 0)
            ch_reset_ctr <= ch_reset_ctr - 1'b1;
    end

    wire mux_rst_n = rst_n & ~ch_changing;

    // =====================================================================
    // Single-channel deserializer (rx_process_mux, N_CH=1)
    // =====================================================================
    wire [3:0]  out_valid;
    wire [47:0] out_word;
    wire [3:0]  out_chsel;

    rx_process_mux #(
        .N_CH         (1),
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
        .rst_n        (mux_rst_n),
        .data_in      (cb_d[channel_sel]),
        .sync_in      (cb_sync),
        .next_amps_in (cb_read),
        .out_valid    (out_valid),
        .out_word     (out_word),
        .out_chsel    (out_chsel)
    );

    // =====================================================================
    // Pack 12-bit samples into 16-bit FIFO words
    // Upper nibble = channel ID for host-side verification
    // =====================================================================
    wire        fifo_full;
    wire        fifo_wen = out_valid[0] & ~fifo_full;
    wire [15:0] fifo_wdata = {channel_sel, out_word[11:0]};

    // =====================================================================
    // Async FIFO: 32 MHz → 100 MHz
    // =====================================================================
    async_fifo #(.W(16), .D(4096)) u_fifo (
        .wclk         (cb_clk32mhz),
        .wrst_n       (rst_n),
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

    // =====================================================================
    // LEDs
    // =====================================================================
    assign led_spi = spi_done;

    reg [21:0] data_stretch = 22'd0;
    always @(posedge cb_clk32mhz) begin
        if (out_valid[0])
            data_stretch <= 22'h3FFFFF;
        else if (data_stretch != 0)
            data_stretch <= data_stretch - 1'b1;
    end
    assign led_data = (data_stretch != 0);

    reg [21:0] usb_stretch = 22'd0;
    always @(posedge ft600_clk) begin
        if (~ft600_wr_n)
            usb_stretch <= 22'h3FFFFF;
        else if (usb_stretch != 0)
            usb_stretch <= usb_stretch - 1'b1;
    end
    assign led_usb = (usb_stretch != 0);

    // =====================================================================
    // Unused input anchors
    // =====================================================================
    wire _unused = &{1'b0, cb_ac_in, cb_imp_test,
                     out_chsel, out_valid[3:1], out_word[47:12], clk32M};

endmodule


// =============================================================================
// PLL: 16 MHz → 32 MHz
// =============================================================================
module pll_16to32 (
    input  wire clk_in,
    output wire clk_out,
    output wire locked
);
    wire vcc = 1'b1, gnd = 1'b0, clk_fb;

    EHXPLLL #(
        .CLKI_DIV     (1),
        .CLKFB_DIV    (2),
        .CLKOP_DIV    (15),
        .CLKOP_ENABLE ("ENABLED"),
        .CLKOS_ENABLE ("DISABLED"),
        .CLKOS2_ENABLE("DISABLED"),
        .CLKOS3_ENABLE("DISABLED"),
        .OUTDIVIDER_MUXA("DIVA"),
        .FEEDBK_PATH  ("CLKOP"),
        .PLLRST_ENA   ("DISABLED")
    ) pll_macro (
        .CLKI(clk_in), .CLKFB(clk_fb),
        .PHASESEL1(gnd), .PHASESEL0(gnd),
        .PHASEDIR(gnd), .PHASESTEP(gnd),
        .PHASELOADREG(gnd), .STDBY(gnd), .RST(gnd),
        .ENCLKOP(vcc), .ENCLKOS(gnd), .ENCLKOS2(gnd), .ENCLKOS3(gnd),
        .CLKOP(clk_fb), .LOCK(locked)
    );

    assign clk_out = clk_fb;
endmodule
