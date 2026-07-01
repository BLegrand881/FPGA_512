//==============================================================================
// UWB_Serial_Handler.v
//
// rx_process_mux  — 8-channel ADC bit-deserializer → RX FIFO → TX FIFO →
//                   8:4 word-level TDM mux.  Outputs 4 word-parallel lanes
//                   (12-bit word + valid strobe per lane per clock).
//                   PRN sync words are removed; framing is handled entirely
//                   by the downstream lane_framer modules.
//
// lane_framer     — Accumulates 128 12-bit words from one TDM output lane,
//                   then serializes the 132-word framed packet:
//
//                     [SYNC(12b)] [LANE_ID(12b)] [CYCLE_CNT(12b)]
//                     [DATA × 128 words (12b each)]
//                     [CRC-12(12b)]
//
//                   Words emitted MSB-first at 1 bit per clock.
//                   New words arriving during emission are dropped (COLLECT
//                   restarts after EMIT completes).
//                   See FRAMING_SPEC.txt for full field definitions.
//
// sync_fifo       — Plain RTL synchronous FIFO (swap for vendor IP).
//==============================================================================


//==============================================================================
// rx_process_mux
//==============================================================================
module rx_process_mux #(
    parameter int N_CH          = 8,
    parameter int N_ADC_PER_GP  = 4,
    parameter int ADC_BITS      = 12,
    parameter int ZERO_CYCLES   = 11,
    parameter int GROUP_CYCLES  = 64,
    parameter int N_GROUPS      = 16,
    parameter int SAMPS_PER_FR  = N_GROUPS * N_ADC_PER_GP,  // 64
    parameter int RX_FIFO_DEPTH = 16,
    parameter int TX_FIFO_DEPTH = 16
)(
    input  wire                clk,
    input  wire                rst_n,

    // 8 independent serial bitstreams (one per ADC channel)
    input  wire [N_CH-1:0]     data_in,
    // sync_in: kept in port for future frame-alignment use; not used internally
    input  wire [N_CH-1:0]     sync_in,
    // next_amps_in: group-boundary pulse, resets the per-channel group counter
    input  wire [N_CH-1:0]     next_amps_in,

    // 4 TDM-muxed output lanes (word-parallel, one 12-bit word per clock when valid)
    // out_word packed as {lane3[11:0], lane2[11:0], lane1[11:0], lane0[11:0]}
    output reg  [3:0]          out_valid,
    output reg  [47:0]         out_word,
    output reg  [3:0]          out_chsel  // 0 = even channel of pair, 1 = odd
);

    // Silence unused-input warning so the IO buffer / pull stays in the netlist
    wire _sync_unused = &{1'b0, sync_in};

    //==========================================================================
    // Stage 1: Per-channel bit deserializer → RX FIFO
    //
    // ADC serial format (per 64-cycle group):
    //   cycles  0..10 : 11 leading zero cycles  (ZERO_CYCLES)
    //   cycles 13..60 : 4 ADC × 12 bits, bit-interleaved
    //                   which_adc = (cyc - 13) % 4
    //                   which_bit = (cyc - 13) / 4
    //   cycles 61..63 : 3 trailing zero cycles  (ignored by in_data_phase guard)
    //
    // MSB of ADC[k] arrives at cycle 13 + 4*11 + k = 57+k.
    // Four RX-FIFO pushes per group (cycles 57..60), one per ADC.
    //==========================================================================
    wire [ADC_BITS-1:0] rx_fifo_wdata [0:N_CH-1];
    wire [N_CH-1:0]     rx_fifo_wen;
    wire [N_CH-1:0]     rx_fifo_full;
    wire [ADC_BITS-1:0] rx_fifo_rdata [0:N_CH-1];
    wire [N_CH-1:0]     rx_fifo_ren;
    wire [N_CH-1:0]     rx_fifo_empty;

    reg [5:0]          cyc_in_group [0:N_CH-1];
    // 4 shift registers per channel — one per interleaved ADC lane.
    // Bits arrive LSB-first; each SR shifts right: {new_bit, sr[11:1]}.
    // After 12 shifts: sr[0]=bit0(LSB) … sr[11]=bit11(MSB).
    reg [ADC_BITS-1:0] sr0 [0:N_CH-1];
    reg [ADC_BITS-1:0] sr1 [0:N_CH-1];
    reg [ADC_BITS-1:0] sr2 [0:N_CH-1];
    reg [ADC_BITS-1:0] sr3 [0:N_CH-1];

    genvar ch;
    generate
        for (ch = 0; ch < N_CH; ch = ch + 1) begin : g_rx
            // Data phase: leading zeros excluded, trailing zeros (61..63) excluded
            wire        in_data_phase = (cyc_in_group[ch] >= ZERO_CYCLES) &&
                                        (cyc_in_group[ch] < ZERO_CYCLES + N_ADC_PER_GP * ADC_BITS);
            wire [5:0]  bit_idx       = cyc_in_group[ch] - ZERO_CYCLES[5:0];
            wire [1:0]  which_adc     = bit_idx[1:0];   // bit_idx % 4
            wire [3:0]  which_bit     = bit_idx[5:2];   // bit_idx / 4

            // Group-cycle counter, re-zeroed by next_amps pulse
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n)
                    cyc_in_group[ch] <= 6'd0;
                else if (next_amps_in[ch])
                    cyc_in_group[ch] <= 6'd0;
                else if (cyc_in_group[ch] != GROUP_CYCLES - 1)
                    cyc_in_group[ch] <= cyc_in_group[ch] + 1'b1;
            end

            // Shift one bit per cycle into the selected ADC's shift register.
            // Case statement avoids runtime-indexed 3D array writes (which
            // cause multiply-driven nets during register flattening).
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    sr0[ch] <= {ADC_BITS{1'b0}};
                    sr1[ch] <= {ADC_BITS{1'b0}};
                    sr2[ch] <= {ADC_BITS{1'b0}};
                    sr3[ch] <= {ADC_BITS{1'b0}};
                end else if (in_data_phase) begin
                    case (which_adc)
                        2'd0: sr0[ch] <= {data_in[ch], sr0[ch][ADC_BITS-1:1]};
                        2'd1: sr1[ch] <= {data_in[ch], sr1[ch][ADC_BITS-1:1]};
                        2'd2: sr2[ch] <= {data_in[ch], sr2[ch][ADC_BITS-1:1]};
                        2'd3: sr3[ch] <= {data_in[ch], sr3[ch][ADC_BITS-1:1]};
                    endcase
                end
            end

            // Push when the last bit of an ADC word has been shifted in.
            wire push_now = in_data_phase && (which_bit == ADC_BITS - 1);

            assign rx_fifo_wen[ch]   = push_now && !rx_fifo_full[ch];
            assign rx_fifo_wdata[ch] = (which_adc == 2'd0) ? {data_in[ch], sr0[ch][ADC_BITS-1:1]} :
                                       (which_adc == 2'd1) ? {data_in[ch], sr1[ch][ADC_BITS-1:1]} :
                                       (which_adc == 2'd2) ? {data_in[ch], sr2[ch][ADC_BITS-1:1]} :
                                                             {data_in[ch], sr3[ch][ADC_BITS-1:1]};

            sync_fifo #(.W(ADC_BITS), .D(RX_FIFO_DEPTH)) u_rx_fifo (
                .clk  (clk),
                .rst_n(rst_n),
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
    // Stage 2: RX FIFO → TX FIFO  (direct transfer; no PRN insertion)
    //
    // Framing (SYNC / LANE_ID / CYCLE_CNT / CRC-12) is inserted downstream
    // by the lane_framer modules, keeping this stage simple.
    //==========================================================================
    wire [ADC_BITS-1:0] tx_fifo_wdata [0:N_CH-1];
    wire [N_CH-1:0]     tx_fifo_wen;
    wire [N_CH-1:0]     tx_fifo_full;
    wire [ADC_BITS-1:0] tx_fifo_rdata [0:N_CH-1];
    reg  [N_CH-1:0]     tx_fifo_ren;
    wire [N_CH-1:0]     tx_fifo_empty;

    generate
        for (ch = 0; ch < N_CH; ch = ch + 1) begin : g_proc
            assign rx_fifo_ren[ch]   = !rx_fifo_empty[ch] && !tx_fifo_full[ch];
            assign tx_fifo_wen[ch]   = rx_fifo_ren[ch];
            assign tx_fifo_wdata[ch] = rx_fifo_rdata[ch];

            sync_fifo #(.W(ADC_BITS), .D(TX_FIFO_DEPTH)) u_tx_fifo (
                .clk  (clk),
                .rst_n(rst_n),
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
    // Stage 3: 8 → 4 word-level TDM mux
    //
    // Lane p muxes channel pair (2p, 2p+1). Alternates between even/odd each
    // cycle. If the preferred channel has no data, tries the other. mux_sel
    // only toggles when the preferred channel was successfully served.
    //==========================================================================
    reg [3:0] mux_sel;

    // effective_empty: a channel with a pending ren (registered, fires next cycle)
    // must be treated as already consumed — its rdata is still the stale head.
    wire [N_CH-1:0] eff_empty = tx_fifo_empty | tx_fifo_ren;

    // Per-lane channel selects (combinatorial, depend on mux_sel)
    wire [2:0] first0  = mux_sel[0] ? 3'd1 : 3'd0;
    wire [2:0] second0 = mux_sel[0] ? 3'd0 : 3'd1;
    wire [2:0] first1  = mux_sel[1] ? 3'd3 : 3'd2;
    wire [2:0] second1 = mux_sel[1] ? 3'd2 : 3'd3;
    wire [2:0] first2  = mux_sel[2] ? 3'd5 : 3'd4;
    wire [2:0] second2 = mux_sel[2] ? 3'd4 : 3'd5;
    wire [2:0] first3  = mux_sel[3] ? 3'd7 : 3'd6;
    wire [2:0] second3 = mux_sel[3] ? 3'd6 : 3'd7;

    // Unrolled 8->4 mux, one section per output lane
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mux_sel     <= 4'd0;
            out_valid   <= 4'd0;
            out_chsel   <= 4'd0;
            tx_fifo_ren <= '0;
            out_word    <= '0;
        end else begin
            tx_fifo_ren <= '0;

            // Lane 0: channels 0 (even) and 1 (odd)
            if (!eff_empty[first0]) begin
                tx_fifo_ren[first0]  <= 1'b1;
                out_word[0 +: 12]    <= tx_fifo_rdata[first0];
                out_valid[0]         <= 1'b1;
                out_chsel[0]         <= mux_sel[0];
                mux_sel[0]           <= ~mux_sel[0];
            end else if (!eff_empty[second0]) begin
                tx_fifo_ren[second0] <= 1'b1;
                out_word[0 +: 12]    <= tx_fifo_rdata[second0];
                out_valid[0]         <= 1'b1;
                out_chsel[0]         <= ~mux_sel[0];
            end else begin
                out_valid[0] <= 1'b0;
            end

            // Lane 1: channels 2 (even) and 3 (odd)
            if (!eff_empty[first1]) begin
                tx_fifo_ren[first1]  <= 1'b1;
                out_word[12 +: 12]   <= tx_fifo_rdata[first1];
                out_valid[1]         <= 1'b1;
                out_chsel[1]         <= mux_sel[1];
                mux_sel[1]           <= ~mux_sel[1];
            end else if (!eff_empty[second1]) begin
                tx_fifo_ren[second1] <= 1'b1;
                out_word[12 +: 12]   <= tx_fifo_rdata[second1];
                out_valid[1]         <= 1'b1;
                out_chsel[1]         <= ~mux_sel[1];
            end else begin
                out_valid[1] <= 1'b0;
            end

            // Lane 2: channels 4 (even) and 5 (odd)
            if (!eff_empty[first2]) begin
                tx_fifo_ren[first2]  <= 1'b1;
                out_word[24 +: 12]   <= tx_fifo_rdata[first2];
                out_valid[2]         <= 1'b1;
                out_chsel[2]         <= mux_sel[2];
                mux_sel[2]           <= ~mux_sel[2];
            end else if (!eff_empty[second2]) begin
                tx_fifo_ren[second2] <= 1'b1;
                out_word[24 +: 12]   <= tx_fifo_rdata[second2];
                out_valid[2]         <= 1'b1;
                out_chsel[2]         <= ~mux_sel[2];
            end else begin
                out_valid[2] <= 1'b0;
            end

            // Lane 3: channels 6 (even) and 7 (odd)
            if (!eff_empty[first3]) begin
                tx_fifo_ren[first3]  <= 1'b1;
                out_word[36 +: 12]   <= tx_fifo_rdata[first3];
                out_valid[3]         <= 1'b1;
                out_chsel[3]         <= mux_sel[3];
                mux_sel[3]           <= ~mux_sel[3];
            end else if (!eff_empty[second3]) begin
                tx_fifo_ren[second3] <= 1'b1;
                out_word[36 +: 12]   <= tx_fifo_rdata[second3];
                out_valid[3]         <= 1'b1;
                out_chsel[3]         <= ~mux_sel[3];
            end else begin
                out_valid[3] <= 1'b0;
            end
        end
    end

endmodule


//==============================================================================
// lane_framer
//
// Accumulates DATA_WORDS (128) 12-bit words from the TDM mux for one lane,
// then serializes a 132-word framed packet MSB-first at 1 bit per clock:
//
//   Word   0      SYNC_WORD  (12b) — unique per-lane sync pattern
//   Word   1      LANE_ID    (12b) — {4'b0,LANE_NUM[1:0]} || ~{4'b0,LANE_NUM[1:0]}
//   Word   2      CYCLE_CNT  (12b) — rolling output-frame counter (wraps at 4096)
//   Words  3..130 DATA       (12b each, 128 words)
//                              even-channel words at positions 3,5,7,...,129
//                              odd-channel  words at positions 4,6,8,...,130
//   Word 131      CRC-12     (12b) — CRC over words 3..130 only
//
// During EMIT (1584 clock cycles), incoming words are ignored.
// COLLECT restarts immediately after EMIT completes.
//
// Parameters
//   SYNC_WORD  — 12-bit sync pattern (DC-balanced, unique per lane)
//   LANE_NUM   — 2-bit lane index (0–3)
//   DATA_WORDS — words per frame body (default 128 = 64 amps × 2 channels)
//==============================================================================
module lane_framer #(
    parameter [11:0] SYNC_WORD  = 12'hA35,
    parameter [1:0]  LANE_NUM   = 2'd0,
    parameter int    DATA_WORDS = 128
)(
    input  wire        clk,
    input  wire        rst_n,
    // Word input from rx_process_mux Stage 3
    input  wire        in_valid,
    input  wire [11:0] in_word,
    // Serialized output: MSB-first, 1 bit per clock
    output reg         serial_out
);

    // Total frame words: SYNC + LANE_ID + CYCLE_CNT + data×128 + CRC = 132
    localparam int    FRAME_WORDS = DATA_WORDS + 4;
    localparam [7:0]  LAST_WPTR   = FRAME_WORDS - 1;   // 8'd131

    // -----------------------------------------------------------------------
    // Data buffer — stores one complete set of DATA_WORDS during COLLECT
    // -----------------------------------------------------------------------
    reg [11:0] dbuf [0:DATA_WORDS-1];
    reg [$clog2(DATA_WORDS)-1:0] collect_cnt;  // [6:0] for DATA_WORDS=128

    // -----------------------------------------------------------------------
    // CRC-12 accumulator
    //   Polynomial : 0x80F  (x^12 + x^11 + x^3 + x^2 + x + 1, ITU-T CRC-12)
    //   Seed       : 12'h000
    //   Covers     : DATA words only (words 3..130); header excluded
    //   Bit order  : MSB of each word processed first (matches serial output)
    // -----------------------------------------------------------------------
    reg [11:0] crc_acc;

    function automatic [11:0] crc12_next;
        input [11:0] crc_in;
        input [11:0] data;
        reg [11:0] c;
        integer i;
        begin
            c = crc_in;
            for (i = 11; i >= 0; i = i - 1) begin
                if (data[i] ^ c[11])
                    c = {c[10:0], 1'b0} ^ 12'h80F;
                else
                    c = {c[10:0], 1'b0};
            end
            crc12_next = c;
        end
    endfunction

    // -----------------------------------------------------------------------
    // Emit state
    // -----------------------------------------------------------------------
    reg        emitting;
    reg [7:0]  emit_wptr;   // 0 .. LAST_WPTR (131)
    reg [3:0]  emit_bptr;   // 11 .. 0  (bit pointer, MSB first)
    reg [11:0] cycle_cnt;   // 12-bit rolling output-frame counter

    // LANE_ID: upper 6 bits = zero-padded lane number, lower 6 bits = complement
    wire [5:0]  lid_hi  = {4'b0000, LANE_NUM};
    wire [11:0] lane_id = {lid_hi, ~lid_hi};

    // -----------------------------------------------------------------------
    // Combinatorial frame word select
    // The registered emit_wptr and crc_acc are valid 1 cycle after EMIT starts,
    // which is correct because serial_out is itself registered one cycle later.
    // -----------------------------------------------------------------------
    reg [11:0] emit_word_data;
    always @* begin
        case (emit_wptr)
            8'd0:    emit_word_data = SYNC_WORD;
            8'd1:    emit_word_data = lane_id;
            8'd2:    emit_word_data = cycle_cnt;
            LAST_WPTR: emit_word_data = crc_acc;            // word 131 = CRC
            default: emit_word_data = dbuf[emit_wptr - 8'd3]; // words 3..130
        endcase
    end

    // -----------------------------------------------------------------------
    // State machine
    // -----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            emitting    <= 1'b0;
            collect_cnt <= '0;
            crc_acc     <= 12'd0;
            cycle_cnt   <= 12'd0;
            emit_wptr   <= 8'd0;
            emit_bptr   <= 4'd11;
            serial_out  <= 1'b0;
        end else if (!emitting) begin
            // ==============================================================
            // COLLECT: accept words one at a time, accumulate CRC
            // ==============================================================
            serial_out <= 1'b0;
            if (in_valid) begin
                dbuf[collect_cnt] <= in_word;
                crc_acc           <= crc12_next(crc_acc, in_word);
                if (collect_cnt == DATA_WORDS - 1) begin
                    // Buffer full — switch to EMIT next cycle
                    collect_cnt <= '0;
                    emitting    <= 1'b1;
                    emit_wptr   <= 8'd0;
                    emit_bptr   <= 4'd11;
                end else begin
                    collect_cnt <= collect_cnt + 1'b1;
                end
            end
        end else begin
            // ==============================================================
            // EMIT: serialize 132 words × 12 bits = 1584 clock cycles
            // ==============================================================
            serial_out <= emit_word_data[emit_bptr];

            if (emit_bptr == 4'd0) begin
                emit_bptr <= 4'd11;
                if (emit_wptr == LAST_WPTR) begin
                    // Last bit of CRC sent — frame complete
                    emitting  <= 1'b0;
                    cycle_cnt <= cycle_cnt + 1'b1;
                    crc_acc   <= 12'd0;   // reset CRC for next frame
                end else begin
                    emit_wptr <= emit_wptr + 1'b1;
                end
            end else begin
                emit_bptr <= emit_bptr - 1'b1;
            end
        end
    end

endmodule


//==============================================================================
// sync_fifo  —  Simple synchronous FIFO (swap for vendor IP at synthesis time)
//==============================================================================
module sync_fifo #(
    parameter int W = 12,
    parameter int D = 16
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         wen,
    input  wire [W-1:0] wdata,
    output wire         full,
    input  wire         ren,
    output wire [W-1:0] rdata,
    output wire         empty
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
            if (wen && !full)  begin mem[wptr[AW-1:0]] <= wdata; wptr <= wptr + 1'b1; end
            if (ren && !empty) begin                              rptr <= rptr + 1'b1; end
        end
    end
endmodule
