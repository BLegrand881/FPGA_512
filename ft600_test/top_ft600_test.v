// =============================================================================
// top_ft600_test.v  —  Standalone FT600 counter-streamer test
// =============================================================================

module top (
    input  wire        clk_16m,      // 16 MHz crystal (A7) — unused, kept for LPF

    // FT600 USB FIFO (U1)
    input  wire        ft600_clk,    // G1  — 100 MHz clock from FT600
    input  wire        ft600_txe_n,  // C1  — TX buffer not full (low = can write)
    input  wire        ft600_rxf_n,  // C2  — RX data available (unused)
    output wire        ft600_wr_n,   // C3  — write strobe (active low)
    output wire        ft600_rd_n,   // B1  — read strobe (active low)
    output wire        ft600_oe_n,   // B2  — output enable (active low)
    output wire        ft600_be0,    // E3  — byte enable 0
    output wire        ft600_be1,    // D3  — byte enable 1
    inout  wire [15:0] ft600_d,      // 16-bit bidirectional data bus

    // Status LEDs
    output wire        led_power,    // P16 — solid on = FPGA alive
    output wire        led_data      // M13
);

    // -------------------------------------------------------------------------
    // 1. Reset synchronizer in ft600_clk domain
    // -------------------------------------------------------------------------
    reg [3:0] ft_rst_pipe = 4'hF;
    always @(posedge ft600_clk) begin
        ft_rst_pipe <= {ft_rst_pipe[2:0], 1'b0};
    end
    wire ft_rst_n = ~ft_rst_pipe[3];

    // -------------------------------------------------------------------------
    // 2. Counter streamer
    // -------------------------------------------------------------------------
    wire [15:0] ft_data_out;
    wire        ft_data_oe;
    wire [1:0]  ft_be;

    ft600_streamer u_ft600 (
        .clk      (ft600_clk),
        .rst_n    (ft_rst_n),
        .txe_n    (ft600_txe_n),
        .wr_n     (ft600_wr_n),
        .rd_n     (ft600_rd_n),
        .oe_n     (ft600_oe_n),
        .be       (ft_be),
        .data_out (ft_data_out),
        .data_oe  (ft_data_oe)
    );

    assign ft600_be0 = ft_be[0];
    assign ft600_be1 = ft_be[1];

    // Bidirectional data bus
    assign ft600_d = ft_data_oe ? ft_data_out : 16'bz;

    // -------------------------------------------------------------------------
    // 3. Status LEDs
    //    D1: always on
    //    D2: latches ON if TXE_N ever goes low (even 1 cycle)
    // -------------------------------------------------------------------------
    assign led_power = 1'b1;

    reg txe_seen = 1'b0;
    always @(posedge ft600_clk)
        if (~ft600_txe_n) txe_seen <= 1'b1;
    assign led_data = txe_seen;

    // -------------------------------------------------------------------------
    // 4. Unused inputs
    // -------------------------------------------------------------------------
    wire _unused_ok = &{1'b0, clk_16m, ft600_rxf_n};

endmodule
