//==============================================================================
// rx_process_mux.v
//
// 8 channels of adc_stream_gen format -> deserialize -> RX FIFO ->
// strip zeros + insert PRN sync -> TX FIFO -> 8:4 word-level TDM mux.
//
// All FIFOs are 12 bits wide. PRN sync is stored as 3 consecutive 12-bit
// entries at the start of each frame.
//==============================================================================

module rx_process_mux #(
    parameter int N_CH          = 8,
    parameter int N_ADC_PER_GP  = 4,
    parameter int ADC_BITS      = 12,
    parameter int ZERO_CYCLES   = 16,
    parameter int GROUP_CYCLES  = 64,
    parameter int N_GROUPS      = 16,
    parameter int SAMPS_PER_FR  = N_GROUPS * N_ADC_PER_GP,   // 64
    parameter int RX_FIFO_DEPTH = 16,
    parameter int TX_FIFO_DEPTH = 16,
    parameter logic [11:0] PRN0 = 12'hCAF,
    parameter logic [11:0] PRN1 = 12'hEF0,
    parameter logic [11:0] PRN2 = 12'h0D5
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // 8 serial inputs, one per adc_stream_gen instance
    input  wire [N_CH-1:0]        data_in,
    input  wire [N_CH-1:0]        sync_in,
    input  wire [N_CH-1:0]        next_amps_in,

    // 4 multiplexed outputs. One 12-bit word per cycle when valid.
    output reg  [3:0]             out_valid,
    output reg  [11:0]            out_word  [0:3],
    output reg  [3:0]             out_chsel    // which channel of the pair (0=even,1=odd)
);

    //==========================================================================
    // Stage 1: Per-channel deserializer
    //==========================================================================
    // adc_stream_gen format per 64-cycle group:
    //   cycles  0..15 : zero padding (skipped)
    //   cycles 16..63 : 48 data bits, bit-interleaved across 4 ADCs
    //                   which_adc = (cyc - 16) % 4
    //                   which_bit = (cyc - 16) / 4
    // ADC k's bit 11 (MSB, last) arrives at cyc = 16 + 4*11 + k = 60+k.
    // So pushes to RX FIFO happen one per cycle at cycles 60,61,62,63.

    wire [ADC_BITS-1:0] rx_fifo_wdata [0:N_CH-1];
    wire [N_CH-1:0]     rx_fifo_wen;
    wire [N_CH-1:0]     rx_fifo_full;
    wire [ADC_BITS-1:0] rx_fifo_rdata [0:N_CH-1];
    reg  [N_CH-1:0]     rx_fifo_ren;
    wire [N_CH-1:0]     rx_fifo_empty;

    reg [5:0]           cyc_in_group [0:N_CH-1];
    reg [ADC_BITS-1:0]  adc_sr       [0:N_CH-1][0:N_ADC_PER_GP-1];

    genvar ch;
    generate
        for (ch = 0; ch < N_CH; ch = ch + 1) begin : g_rx
            wire        in_data_phase = (cyc_in_group[ch] >= ZERO_CYCLES);
            wire [5:0]  bit_idx       = cyc_in_group[ch] - ZERO_CYCLES[5:0];
            wire [1:0]  which_adc     = bit_idx[1:0];     // bit_idx % 4
            wire [3:0]  which_bit     = bit_idx[5:2];     // bit_idx / 4

            // Group cycle counter, resynced by next_amps pulse
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n)
                    cyc_in_group[ch] <= 6'd0;
                else if (next_amps_in[ch])
                    cyc_in_group[ch] <= 6'd0;
                else if (cyc_in_group[ch] != GROUP_CYCLES-1)
                    cyc_in_group[ch] <= cyc_in_group[ch] + 1'b1;
            end

            // Deserialize one bit per cycle into the correct ADC shift register
            integer k;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    for (k = 0; k < N_ADC_PER_GP; k = k + 1)
                        adc_sr[ch][k] <= '0;
                end else if (in_data_phase) begin
                    adc_sr[ch][which_adc][which_bit] <= data_in[ch];
                end
            end

            // Push one ADC sample at cycles 60..63 (one per ADC, naturally serialized)
            wire push_now = in_data_phase &&
                            (which_bit == ADC_BITS-1) &&
                            (cyc_in_group[ch] >= 60);

            assign rx_fifo_wen[ch]   = push_now && !rx_fifo_full[ch];
            assign rx_fifo_wdata[ch] = adc_sr[ch][which_adc];

            sync_fifo #(.W(ADC_BITS), .D(RX_FIFO_DEPTH)) u_rx_fifo (
                .clk(clk), .rst_n(rst_n),
                .wen  (rx_fifo_wen[ch]),
                .wdata(rx_fifo_wdata[ch]),
                .full (rx_fifo_full[ch]),
                .ren  (rx_fifo_ren[ch]),
                .rdata(rx_fifo_rdata[ch]),
                .empty(rx_fifo_empty[ch])
            );
        end
    endgenerate

    //==========================================================================
    // Stage 2: Strip zeros + insert PRN sync -> TX FIFO
    //==========================================================================
    // Zeros are already stripped (deserializer never pushed them).
    // For each channel: at the start of every frame, push PRN0,PRN1,PRN2 into
    // the TX FIFO before pushing the 64 ADC samples for that frame.
    //
    // State machine per channel:
    //   prn_idx counts 0..3. 0..2 = push PRN0..PRN2, 3 = push samples.
    //   When 64 samples pushed, wrap back to 0 for next frame's PRN.

    wire [ADC_BITS-1:0] tx_fifo_wdata [0:N_CH-1];
    wire [N_CH-1:0]     tx_fifo_wen;
    wire [N_CH-1:0]     tx_fifo_full;
    wire [ADC_BITS-1:0] tx_fifo_rdata [0:N_CH-1];
    reg  [N_CH-1:0]     tx_fifo_ren;
    wire [N_CH-1:0]     tx_fifo_empty;

    reg [1:0]   prn_idx       [0:N_CH-1];   // 0..2 = PRN chunk, 3 = streaming samples
    reg [5:0]   samp_in_frame [0:N_CH-1];   // 0..63

    generate
        for (ch = 0; ch < N_CH; ch = ch + 1) begin : g_proc
            reg push_prn, push_samp;
            reg [ADC_BITS-1:0] wdata_mux;

            always @* begin
                push_prn  = 1'b0;
                push_samp = 1'b0;
                wdata_mux = '0;
                rx_fifo_ren[ch] = 1'b0;

                if (!tx_fifo_full[ch]) begin
                    if (prn_idx[ch] != 2'd3) begin
                        // Emit PRN chunk
                        push_prn = 1'b1;
                        case (prn_idx[ch])
                            2'd0: wdata_mux = PRN0;
                            2'd1: wdata_mux = PRN1;
                            2'd2: wdata_mux = PRN2;
                            default: wdata_mux = '0;
                        endcase
                    end else if (!rx_fifo_empty[ch]) begin
                        // Pop a sample from RX FIFO and push to TX FIFO
                        push_samp        = 1'b1;
                        rx_fifo_ren[ch]  = 1'b1;
                        wdata_mux        = rx_fifo_rdata[ch];
                    end
                end
            end

            assign tx_fifo_wen[ch]   = push_prn | push_samp;
            assign tx_fifo_wdata[ch] = wdata_mux;

            reg sync_in_d;   // note: per-channel because we're inside the generate loop

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    prn_idx[ch]       <= 2'd0;
                    samp_in_frame[ch] <= 6'd0;
                    sync_in_d         <= 1'b0;
                end else begin
                    sync_in_d <= sync_in[ch];

                    if (sync_in[ch] && !sync_in_d) begin
                        // Rising edge of sync: authoritative frame boundary
                        prn_idx[ch]       <= 2'd0;
                        samp_in_frame[ch] <= 6'd0;
                    end else begin
                        if (push_prn) begin
                            prn_idx[ch] <= prn_idx[ch] + 1'b1;
                        end
                        if (push_samp) begin
                            if (samp_in_frame[ch] == SAMPS_PER_FR-1) begin
                                samp_in_frame[ch] <= 6'd0;
                                prn_idx[ch]       <= 2'd0;
                            end else begin
                                samp_in_frame[ch] <= samp_in_frame[ch] + 1'b1;
                            end
                        end
                    end
                end
            end

            sync_fifo #(.W(ADC_BITS), .D(TX_FIFO_DEPTH)) u_tx_fifo (
                .clk(clk), .rst_n(rst_n),
                .wen  (tx_fifo_wen[ch]),
                .wdata(tx_fifo_wdata[ch]),
                .full (tx_fifo_full[ch]),
                .ren  (tx_fifo_ren[ch]),
                .rdata(tx_fifo_rdata[ch]),
                .empty(tx_fifo_empty[ch])
            );
        end
    endgenerate

    //==========================================================================
    // Stage 3: 8 -> 4 mux. Pair (2p, 2p+1) -> output p. Word-level TDM,
    // alternating. If the scheduled channel has no data, take the other.
    //==========================================================================
    reg [3:0] mux_sel;   // for each output, which channel to try first next cycle

    integer p;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mux_sel     <= 4'd0;
            out_valid   <= 4'd0;
            out_chsel   <= 4'd0;
            tx_fifo_ren <= '0;
            for (p = 0; p < 4; p = p + 1) out_word[p] <= '0;
        end else begin
            tx_fifo_ren <= '0;
            for (p = 0; p < 4; p = p + 1) begin
                automatic int ch_a   = 2*p;
                automatic int ch_b   = 2*p + 1;
                automatic int first  = mux_sel[p] ? ch_b : ch_a;
                automatic int second = mux_sel[p] ? ch_a : ch_b;

                if (!tx_fifo_empty[first]) begin
                    tx_fifo_ren[first] <= 1'b1;
                    out_word[p]        <= tx_fifo_rdata[first];
                    out_valid[p]       <= 1'b1;
                    out_chsel[p]       <= (first == ch_b);
                    mux_sel[p]         <= ~mux_sel[p];
                end else if (!tx_fifo_empty[second]) begin
                    tx_fifo_ren[second] <= 1'b1;
                    out_word[p]         <= tx_fifo_rdata[second];
                    out_valid[p]        <= 1'b1;
                    out_chsel[p]        <= (second == ch_b);
                    // don't toggle mux_sel; preferred channel was empty
                end else begin
                    out_valid[p] <= 1'b0;
                end
            end
        end
    end

endmodule


//==============================================================================
// Simple synchronous FIFO. Swap for vendor IP at synthesis time.
//==============================================================================
module sync_fifo #(
    parameter int W = 12,
    parameter int D = 16
)(
    input  wire           clk,
    input  wire           rst_n,
    input  wire           wen,
    input  wire [W-1:0]   wdata,
    output wire           full,
    input  wire           ren,
    output wire [W-1:0]   rdata,
    output wire           empty
);
    localparam int AW = $clog2(D);
    reg [W-1:0] mem [0:D-1];
    reg [AW:0]  wptr, rptr;

    assign empty = (wptr == rptr);
    assign full  = (wptr[AW] != rptr[AW]) &&
                   (wptr[AW-1:0] == rptr[AW-1:0]);
    assign rdata = mem[rptr[AW-1:0]];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wptr <= '0;
            rptr <= '0;
        end else begin
            if (wen && !full) begin
                mem[wptr[AW-1:0]] <= wdata;
                wptr <= wptr + 1'b1;
            end
            if (ren && !empty) begin
                rptr <= rptr + 1'b1;
            end
        end
    end
endmodule