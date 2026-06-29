# FPGA System Overview — 8-Channel ADC Serial Receiver

This document describes the complete data path from ADC serial input through the FPGA to
serialized output. It is intended to give a new reader a full understanding of what the
system does at every stage.

---

## Hardware Context

- **FPGA**: LFE5U-25F-7BG256I (Lattice ECP5, 25k LUTs, BGA-256 package)
- **Clock**: 12 MHz from FTDI (eval board) or 16 MHz oscillator (custom board) via A7
- **ADC board**: Connected via CN1 board-to-board connector, supplies 8 differential
  serial channels and a 32 MHz decode clock
- **Output**: 4 serial lanes to a downstream receiver (logic analyzer / UWB chip)

---

## Stage 0 — ADC Serial Input Protocol

The ADC board drives 8 independent single-bit serial lines (`data_in[7:0]`) plus a
group-boundary pulse (`next_amps_in[7:0]`) per channel. The decode clock is 32 MHz
(`j33_clk32mhz`), giving a bit period of ~31.25 ns.

Each **64-cycle group** carries samples from 4 ADC sub-channels:

```
Cycle   0 .. 12   — 13 leading zeros       (ZERO_CYCLES = 13)
Cycle  13 .. 60   — 48 data bits           (4 ADCs × 12 bits, bit-interleaved)
Cycle  61 .. 63   — 3 trailing zeros       (ignored)
```

**Bit interleaving** within the data phase (cycles 13–60):

| Cycle | which_adc = (cyc-13) % 4 | which_bit = (cyc-13) / 4 |
|-------|--------------------------|--------------------------|
|  13   | ADC 0                    | bit 0  (LSB)             |
|  14   | ADC 1                    | bit 0                    |
|  15   | ADC 2                    | bit 0                    |
|  16   | ADC 3                    | bit 0                    |
|  17   | ADC 0                    | bit 1                    |
| ...   | ...                      | ...                      |
|  57   | ADC 0                    | bit 11 (MSB)             |
|  58   | ADC 1                    | bit 11                   |
|  59   | ADC 2                    | bit 11                   |
|  60   | ADC 3                    | bit 11                   |

Bits arrive **LSB-first**. The FPGA uses a right-shift accumulator so the word
is naturally assembled: after 12 shifts, `sr[0]` = bit 0 (LSB), `sr[11]` = bit 11 (MSB).

The `next_amps_in` pulse resets the 6-bit cycle counter at the start of each new group,
keeping the FPGA in sync with the ADC board.

---

## Stage 1 — Bit Deserializer (rx_process_mux, Stage 1)

**Module**: `rx_process_mux` in `fpga/UWB_Serial_Handler.v`

For each of the 8 serial channels independently:

1. A 6-bit counter (`cyc_in_group[ch]`) tracks position within the 64-cycle group,
   reset by `next_amps_in`.

2. Four 12-bit shift registers (`sr0`–`sr3`, one per interleaved ADC) accumulate bits:
   ```
   sr[which_adc][ch] <= {data_in[ch], sr[which_adc][ch][11:1]};
   ```
   New bits shift in from the MSB end, so the last shift produces the complete
   12-bit ADC word naturally.

3. A push fires when `which_bit == 11` (last bit of the 12-bit word) and the channel
   is in the data phase. This deposits the completed word into the per-channel **RX FIFO**
   (depth 16, width 12). Four pushes happen per group, one per ADC sub-channel (cycles 57–60).

Each of the 8 channels has its own RX FIFO, giving 8 independent word-rate queues.

---

## Stage 2 — RX FIFO → TX FIFO Passthrough (rx_process_mux, Stage 2)

Each channel drains its RX FIFO directly into a corresponding **TX FIFO** (also depth 16,
width 12):

```
rx_fifo_ren[ch]   = !rx_fifo_empty[ch] && !tx_fifo_full[ch];
tx_fifo_wen[ch]   = rx_fifo_ren[ch];
tx_fifo_wdata[ch] = rx_fifo_rdata[ch];
```

This stage is intentionally minimal — no data transformation occurs. Framing headers
(SYNC, LANE_ID, CYCLE_CNT, CRC-12) are inserted downstream by the `lane_framer` modules.
The two-FIFO buffer absorbs timing jitter between the ADC bit clock domain and the
downstream word-rate consumer.

---

## Stage 3 — 8:4 TDM Mux (rx_process_mux, Stage 3)

The 8 TX FIFOs feed a word-level **time-division multiplexer** that produces 4 output
lanes. Each lane carries data from one channel pair:

| Lane | Even channel | Odd channel |
|------|-------------|-------------|
|  0   | Channel 0   | Channel 1   |
|  1   | Channel 2   | Channel 3   |
|  2   | Channel 4   | Channel 5   |
|  3   | Channel 6   | Channel 7   |

Each lane has a 1-bit selector (`mux_sel[lane]`) that alternates between even and odd
channel priority. Each clock cycle, if the preferred channel has data the lane outputs it
and toggles `mux_sel`; if not, it tries the other channel (without toggling). If neither
has data, `out_valid[lane]` goes low for that cycle.

Outputs per clock:
- `out_valid[3:0]` — one bit per lane, high when a valid word is on that lane
- `out_word[47:0]` — packed 12-bit words: bits `[0+:12]`=lane0, `[12+:12]`=lane1, etc.
- `out_chsel[3:0]` — 0 = even channel served, 1 = odd channel served

---

## Stage 4 — Frame Assembly and Serialization (lane_framer)

**Module**: `lane_framer` in `fpga/UWB_Serial_Handler.v`
One instance per lane, parameterized with a unique `SYNC_WORD` and `LANE_NUM`.

### State machine: COLLECT → EMIT

**COLLECT**: Accepts incoming 12-bit words one at a time. Stores them in a 128-word data
buffer (`dbuf`) and accumulates a running CRC-12. When 128 words have been collected,
transitions immediately to EMIT. Any words arriving during EMIT are dropped.

**EMIT**: Serializes a complete 132-word frame MSB-first at 1 bit per clock (1584 clock
cycles total), then returns to COLLECT for the next frame.

### Frame Structure

```
Word   0       SYNC_WORD   12 bits  — unique per-lane sync pattern
Word   1       LANE_ID     12 bits  — lane number with complement check
Word   2       CYCLE_CNT   12 bits  — rolling frame counter (0–4095, wraps)
Words  3–130   DATA        128 × 12 bits = 1536 bits
Word 131       CRC-12      12 bits  — integrity check over words 3–130 only
─────────────────────────────────────
Total: 132 words × 12 bits = 1584 bits per frame
```

### Field Details

**SYNC_WORD** — DC-balanced (6 ones, 6 zeros), unique per lane:

| Lane | Hex   | Binary           |
|------|-------|-----------------|
|  0   | 0xA35 | 1010_0011_0101  |
|  1   | 0xB46 | 1011_0100_0110  |
|  2   | 0xC57 | 1100_0101_0111  |
|  3   | 0xD68 | 1101_0110_1000  |

**LANE_ID** — encodes lane number with a redundancy check:
```
bits[11:6] = {4'b0000, LANE_NUM}    (zero-extended lane index)
bits[5:0]  = ~bits[11:6]            (bitwise complement)
```
The receiver verifies `bits[5:0] == ~bits[11:6]`; a mismatch flags a header error.

**CYCLE_CNT** — 12-bit counter incremented once per frame. The receiver uses this to
detect dropped frames and determine position within the amplifier scan cycle.

**DATA block** (words 3–130) — 64 amplifier steps × 2 channels per lane, interleaved:
```
Word  3  → even channel, amplifier step 0
Word  4  → odd  channel, amplifier step 0
Word  5  → even channel, amplifier step 1
...
Word 129 → even channel, amplifier step 63
Word 130 → odd  channel, amplifier step 63
```

**CRC-12** — ITU-T CRC-12 computed MSB-first over the 128 data words only:
- Polynomial: `0x80F` (x¹² + x¹¹ + x³ + x² + x + 1)
- Seed: `0x000`
- Detects all 1-, 2-, 3-bit errors and all burst errors ≤ 12 bits (100% coverage)

---

## Physical Output

Data exits the FPGA as 4 independent serial bit streams:

| Signal       | FPGA Pin | Description                        |
|-------------|----------|------------------------------------|
| `la_clk`    | K2       | 32 MHz clock (passthrough of ADC decode clock) |
| `la_data[0]`| A15      | Lane 0 serial stream (channels 0 & 1) |
| `la_data[1]`| F1       | Lane 1 serial stream (channels 2 & 3) |
| `la_data[2]`| H2       | Lane 2 serial stream (channels 4 & 5) |
| `la_data[3]`| G1       | Lane 3 serial stream (channels 6 & 7) |

- Data launches on the **rising** edge of `la_clk`
- Receiver samples on the **falling** edge
- Bit order: **MSB first**

---

## Timing Summary

| Parameter         | Value                          |
|-------------------|-------------------------------|
| ADC bit clock     | ~21.3 MHz (32 MHz / 1.5, from ADC board group structure) |
| Decode clock      | 32 MHz (`j33_clk32mhz`)        |
| Bits per group    | 64 cycles (13 zeros + 48 data + 3 trailing) |
| ADC words per group | 4 (one per interleaved sub-channel) |
| Frame size        | 1584 bits (132 words × 12 bits) |
| Frame rate        | 32 MHz / 1584 ≈ **20.2 kHz**   |
| Latency           | One frame (128 words collected before first bit emitted) |

---

## Receiver Lock Sequence

A downstream receiver locks to the stream as follows:

1. Scan incoming 12-bit words for a known SYNC pattern (0xA35, 0xB46, 0xC57, or 0xD68)
2. On a candidate match, verify Word+1 is a valid LANE_ID (complement check)
3. Record CYCLE_CNT from Word+2
4. Verify CRC-12 at Word+131
5. On the next frame, confirm SYNC reappears at Word+132, LANE_ID is unchanged,
   and CYCLE_CNT incremented by exactly 1
6. Lock declared — begin decoding the data block using word-position mapping

False lock probability ≈ 9 × 10⁻¹⁰ (effectively zero).

---

## Source Files

| File                          | Contents                                      |
|-------------------------------|-----------------------------------------------|
| `fpga/UWB_Serial_Handler.v`   | `rx_process_mux`, `lane_framer`, `sync_fifo` |
| `fpga/top.v`                  | Top-level: clock, reset, I/O connections      |
| `fpga/clock.lpf`              | Pin constraints (J32 ADC in, J33 control, J40 output) |
| `docs/FRAMING_SPEC.txt`       | Detailed frame field specification            |
