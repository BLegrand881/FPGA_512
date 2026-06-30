#!/usr/bin/env python3
"""
receiver.py  —  Software receiver for UWB_Serial_Handler 4-lane serial output.

Reads a Digilent WaveForms logic-analyzer CSV with la_clk + up to 4 serial
data lanes (J40 output), decodes the lane_framer framing protocol, and writes
decoded ADC samples to a CSV file.

Protocol (FRAMING_SPEC.txt / UWB_Serial_Handler.v / lane_framer):
    Bit order  : MSB-first, 1 bit per 32 MHz clock cycle
    Frame      : 132 words × 12 bits = 1584 bits per lane
      Word  0       SYNC_WORD  — 0xA35/0xB46/0xC57/0xD68 (per lane)
      Word  1       LANE_ID    — {6b lane_num, ~6b lane_num}
      Word  2       CYCLE_CNT  — 12-bit rolling frame counter
      Words 3-130   DATA       — 128 × 12-bit ADC words
      Word  131     CRC-12     — ITU-T 0x80F, covers words 3-130 only
    Clock      : data launches on rising la_clk; sample on falling edge

Data layout (lane L, word index w in 3..130):
    pair   = w - 3           → 0..127
    amp_f  = pair // 2       → 0..63
    is_odd = pair  % 2       → 0=even ch, 1=odd ch
    group  = amp_f // 4      → 0..15  (amp_csv = group+1, 1-indexed)
    adc    = amp_f  % 4      → 0..3   (adc_csv = adc+1,  1-indexed)
    D_ch   = 2*L + is_odd + 1 → 1..8

Output CSV matches analysis/sine-all-5_decoded.csv format:
    frame_time, D1_adc1_amp1, ..., D8_adc4_amp16   (512 ADC values per row)

Usage:
    python receiver.py capture.csv
    python receiver.py capture.csv -o decoded.csv
    python receiver.py capture.csv --clk la_clk --data la_d0 la_d1 la_d2 la_d3
    python receiver.py capture.csv --clk serial --lanes 0   # single-lane
    python receiver.py capture.csv --edge rising            # sample on rising
"""

import sys
import csv
import argparse
from pathlib import Path

try:
    import numpy as np
except ImportError:
    sys.exit("ERROR: numpy not found.  Run: pip install numpy  or  conda install numpy")

# =============================================================================
# Protocol constants
# =============================================================================
SYNC_WORDS  = [0xA35, 0xB46, 0xC57, 0xD68]
DATA_WORDS  = 128
FRAME_WORDS = DATA_WORDS + 4   # 132
CRC_POLY    = 0x80F
CRC_SEED    = 0x000

# States
HUNT, ALIGN, PRE_LOCK, LOCKED = 0, 1, 2, 3
STATE_NAMES = {HUNT: "HUNT", ALIGN: "ALIGN", PRE_LOCK: "PRE_LOCK", LOCKED: "LOCKED"}

# Lane → D-channel mapping:  lane L → even=2L+1, odd=2L+2
def d_channel(lane, is_odd):
    return 2 * lane + is_odd + 1   # 1..8

# Build output column name from lane + word position
def col_name(lane, w):
    pair   = w - 3
    amp_f  = pair // 2
    is_odd = pair  % 2
    group  = amp_f // 4
    adc    = amp_f  % 4
    return f"D{d_channel(lane, is_odd)}_adc{adc+1}_amp{group+1}"

# Pre-build ordered column list matching sine-all-5_decoded.csv
ALL_COLS = []
_seen = set()
for _lane in range(4):
    for _w in range(3, 131):
        _c = col_name(_lane, _w)
        if _c not in _seen:
            _seen.add(_c)
            ALL_COLS.append(_c)

# =============================================================================
# CRC-12  (ITU-T, polynomial 0x80F, MSB-first per-word processing)
# Mirrors crc12_next() in lane_framer exactly.
# =============================================================================
def crc12_update(crc, word):
    for i in range(11, -1, -1):
        if ((word >> i) & 1) ^ ((crc >> 11) & 1):
            crc = ((crc << 1) & 0xFFF) ^ CRC_POLY
        else:
            crc = (crc << 1) & 0xFFF
    return crc

# =============================================================================
# Per-lane decoder
# =============================================================================
class LaneDecoder:
    def __init__(self, lane_num):
        self.lane       = lane_num
        self.sync_word  = SYNC_WORDS[lane_num]
        lid_hi          = lane_num & 0x3F          # 6-bit lane number
        self.exp_lid    = (lid_hi << 6) | ((~lid_hi) & 0x3F)

        self._reset_state()

        # Decoded frame accumulator
        self.frames     = []      # list of dicts {cycle_cnt, data[128], time}

        # Error tallies
        self.n_sync_loss = 0
        self.n_lane_id   = 0
        self.n_crc       = 0
        self.n_seq       = 0
        self.n_decoded   = 0

    # ------------------------------------------------------------------ #
    def _reset_state(self):
        self.state    = HUNT
        self.sreg     = 0       # 12-bit hunt shift register
        self.cur_word = 0       # current word assembler (MSB-first)
        self.bptr     = 11      # bit pointer: 11=first bit, 0=last bit
        self.wptr     = 0       # word index in frame (0..131)
        self.crc_acc  = CRC_SEED
        self.dbuf     = [0] * DATA_WORDS
        self.rx_cnt   = 0       # received CYCLE_CNT for this frame
        self.exp_cnt  = 0       # expected CYCLE_CNT for next frame
        self.cur_time = 0.0     # timestamp of frame start (SYNC word arrival)

    # ------------------------------------------------------------------ #
    def feed_bit(self, bit, timestamp=0.0):
        if self.state == HUNT:
            self.sreg = ((self.sreg << 1) | bit) & 0xFFF
            if self.sreg == self.sync_word:
                self.state    = ALIGN
                self.wptr     = 1          # SYNC done, start at LANE_ID
                self.bptr     = 11
                self.cur_word = 0
                self.crc_acc  = CRC_SEED
                self.cur_time = timestamp
        else:
            # Shift bit MSB-first into word assembler
            self.cur_word = ((self.cur_word << 1) | bit) & 0xFFF

            if self.bptr > 0:
                self.bptr -= 1
            else:
                # Word complete — process it
                word       = self.cur_word
                self.bptr  = 11
                self._on_word(word, timestamp)

    # ------------------------------------------------------------------ #
    def _on_word(self, word, ts):
        s = self.state
        if   s == ALIGN:    self._align(word, ts)
        elif s == PRE_LOCK: self._prelock(word, ts)
        elif s == LOCKED:   self._locked(word, ts)

    # ------------------------------------------------------------------ #
    def _align(self, word, ts):
        w = self.wptr
        if w == 1:                          # LANE_ID
            if word != self.exp_lid:
                self.n_lane_id += 1
                self._hunt()
            else:
                self.wptr += 1
        elif w == 2:                        # CYCLE_CNT
            self.rx_cnt  = word
            self.exp_cnt = (word + 1) & 0xFFF
            self.wptr   += 1
        elif 3 <= w <= 130:                 # DATA
            self.dbuf[w - 3] = word
            self.crc_acc     = crc12_update(self.crc_acc, word)
            self.wptr       += 1
        elif w == 131:                      # CRC
            if word != self.crc_acc:
                self.n_crc += 1
                self._hunt()
            else:
                self.state   = PRE_LOCK
                self.wptr    = 0
                self.crc_acc = CRC_SEED

    # ------------------------------------------------------------------ #
    def _prelock(self, word, ts):
        w = self.wptr
        if w == 0:                          # SYNC
            if word != self.sync_word:
                self.n_sync_loss += 1
                self._hunt()
            else:
                self.cur_time = ts
                self.wptr    += 1
        elif w == 1:                        # LANE_ID
            if word != self.exp_lid:
                self.n_lane_id += 1
                self._hunt()
            else:
                self.wptr += 1
        elif w == 2:                        # CYCLE_CNT
            if word != self.exp_cnt:
                self.n_seq += 1
                self._hunt()
            else:
                self.rx_cnt  = word
                self.exp_cnt = (word + 1) & 0xFFF
                self.wptr   += 1
        elif 3 <= w <= 130:                 # DATA
            self.dbuf[w - 3] = word
            self.crc_acc     = crc12_update(self.crc_acc, word)
            self.wptr       += 1
        elif w == 131:                      # CRC — lock confirmed
            if word != self.crc_acc:
                self.n_crc += 1
                self._hunt()
            else:
                self._emit_frame()
                self.state   = LOCKED
                self.wptr    = 0
                self.crc_acc = CRC_SEED

    # ------------------------------------------------------------------ #
    def _locked(self, word, ts):
        w = self.wptr
        if w == 0:                          # SYNC
            if word != self.sync_word:
                self.n_sync_loss += 1
                self._hunt()
            else:
                self.cur_time = ts
                self.wptr    += 1
        elif w == 1:                        # LANE_ID
            if word != self.exp_lid:
                self.n_lane_id += 1
                self._hunt()
            else:
                self.wptr += 1
        elif w == 2:                        # CYCLE_CNT
            if word != self.exp_cnt:
                self.n_seq  += 1
                self.exp_cnt = (word + 1) & 0xFFF   # resync, stay locked
            else:
                self.exp_cnt = (word + 1) & 0xFFF
            self.rx_cnt  = word
            self.wptr   += 1
        elif 3 <= w <= 130:                 # DATA
            self.dbuf[w - 3] = word
            self.crc_acc     = crc12_update(self.crc_acc, word)
            self.wptr       += 1
        elif w == 131:                      # CRC
            if word != self.crc_acc:
                self.n_crc += 1
                self._hunt()            # per spec: re-hunt on CRC fail
            else:
                self._emit_frame()
                self.wptr    = 0
                self.crc_acc = CRC_SEED

    # ------------------------------------------------------------------ #
    def _hunt(self):
        self.state    = HUNT
        self.sreg     = 0
        self.cur_word = 0
        self.bptr     = 11

    # ------------------------------------------------------------------ #
    def _emit_frame(self):
        self.n_decoded += 1
        self.frames.append({
            'lane':      self.lane,
            'cycle_cnt': self.rx_cnt,
            'time':      self.cur_time,
            'data':      list(self.dbuf),
        })

# =============================================================================
# CSV loader — Digilent WaveForms format (# comment header, then header row)
# =============================================================================
def load_waveforms_csv(path):
    """
    Returns (header_list, rows_as_dict_list).
    Skips lines starting with '#'.
    """
    with open(path, newline='') as f:
        lines = [l for l in f if not l.startswith('#') and l.strip()]
    reader = csv.DictReader(lines)
    rows   = list(reader)
    return reader.fieldnames, rows

# =============================================================================
# Main
# =============================================================================
def main():
    parser = argparse.ArgumentParser(
        description="Decode UWB_Serial_Handler 4-lane serial output from a "
                    "Digilent WaveForms logic-analyzer CSV.")

    parser.add_argument("input",
        help="Input CSV file (Digilent WaveForms logic-analyzer capture)")
    parser.add_argument("-o", "--output", default=None,
        help="Output CSV file (default: <input>_decoded.csv)")
    parser.add_argument("--clk", default="la_clk",
        help="Column name for the serial clock (default: la_clk)")
    parser.add_argument("--data", nargs="+", default=None,
        metavar="COL",
        help="Column name(s) for serial data lanes, in lane order "
             "(default: la_data_0 la_data_1 la_data_2 la_data_3). "
             "Provide 1 column for single-lane capture.")
    parser.add_argument("--lanes", nargs="+", type=int, default=None,
        metavar="N",
        help="Lane numbers (0-3) corresponding to each --data column. "
             "Default: 0 1 2 3 (or just 0 for a single column).")
    parser.add_argument("--edge", choices=["falling", "rising"], default="falling",
        help="Clock edge to sample data on (default: falling, per spec).")
    parser.add_argument("--time-col", default="Time (s)",
        help="Column name for the timestamp (default: 'Time (s)')")
    parser.add_argument("--list-cols", action="store_true",
        help="Print available column names and exit.")
    parser.add_argument("--raw", action="store_true",
        help="Output raw 12-bit hex words instead of decimal ADC counts.")

    args = parser.parse_args()

    # ------------------------------------------------------------------
    # Load CSV
    # ------------------------------------------------------------------
    in_path = Path(args.input)
    if not in_path.exists():
        sys.exit(f"ERROR: file not found: {in_path}")

    print(f"Loading {in_path} ...", end=" ", flush=True)
    fieldnames, rows = load_waveforms_csv(in_path)
    print(f"{len(rows):,} samples")

    if args.list_cols:
        print("Available columns:")
        for c in (fieldnames or []):
            print(f"  {c!r}")
        return

    if not rows:
        sys.exit("ERROR: no data rows found")

    time_col = args.time_col

    # ------------------------------------------------------------------
    # Resolve data columns and lane numbers
    # ------------------------------------------------------------------
    data_cols = args.data or ["la_data_0", "la_data_1", "la_data_2", "la_data_3"]
    lane_nums = args.lanes or list(range(len(data_cols)))

    if len(data_cols) != len(lane_nums):
        sys.exit("ERROR: --data and --lanes must have the same number of entries")

    # Validate columns exist
    missing = [c for c in [args.clk] + data_cols + [time_col]
               if c not in (fieldnames or [])]
    if missing:
        # Show available columns to help user
        print(f"ERROR: column(s) not found: {missing}")
        print("Available columns:")
        for c in (fieldnames or []):
            print(f"  {c!r}")
        sys.exit(1)

    # ------------------------------------------------------------------
    # Extract and convert to numpy arrays for fast edge detection
    # ------------------------------------------------------------------
    print("Parsing signals ...", end=" ", flush=True)
    times = np.array([float(r[time_col])  for r in rows])
    clk   = np.array([int(r[args.clk])    for r in rows], dtype=np.uint8)
    data  = [np.array([int(r[c]) for r in rows], dtype=np.uint8)
             for c in data_cols]
    print("done")

    # ------------------------------------------------------------------
    # Detect sample edges
    # ------------------------------------------------------------------
    diff = np.diff(clk.astype(np.int8))
    if args.edge == "falling":
        edge_idx = np.where(diff < 0)[0] + 1     # 1→0 transitions
    else:
        edge_idx = np.where(diff > 0)[0] + 1     # 0→1 transitions

    print(f"Found {len(edge_idx):,} {args.edge} clock edges")

    if len(edge_idx) == 0:
        sys.exit("ERROR: no clock edges found — check --clk column name and --edge setting")

    # Sample each data lane at every edge
    sampled = [d[edge_idx] for d in data]
    t_edge  = times[edge_idx]

    # ------------------------------------------------------------------
    # Run per-lane decoders
    # ------------------------------------------------------------------
    decoders = {ln: LaneDecoder(ln) for ln in lane_nums}

    print(f"Decoding {len(lane_nums)} lane(s): {lane_nums} ...")
    for i in range(len(edge_idx)):
        ts = float(t_edge[i])
        for j, ln in enumerate(lane_nums):
            decoders[ln].feed_bit(int(sampled[j][i]), ts)

    # ------------------------------------------------------------------
    # Results summary
    # ------------------------------------------------------------------
    total_frames = sum(d.n_decoded for d in decoders.values())
    print(f"\n{'='*60}")
    print(f"  DECODING COMPLETE  —  {total_frames} frame(s) total")
    print(f"{'='*60}")
    for ln in lane_nums:
        d = decoders[ln]
        sync_s = SYNC_WORDS[ln]
        state  = STATE_NAMES[d.state]
        print(f"\n  Lane {ln}  (SYNC=0x{sync_s:03X}, "
              f"D{2*ln+1}/D{2*ln+2}, final state: {state})")
        print(f"    Frames decoded : {d.n_decoded}")
        print(f"    SYNC losses    : {d.n_sync_loss}"
              + (" ← check bit alignment" if d.n_sync_loss > 0 else ""))
        print(f"    LANE_ID errors : {d.n_lane_id}")
        print(f"    CRC errors     : {d.n_crc}")
        print(f"    Seq gaps       : {d.n_seq}"
              + (" ← dropped frames" if d.n_seq > 0 else ""))
    print()

    if total_frames == 0:
        print("WARNING: no frames decoded.")
        print("  • Verify --clk and --data column names (use --list-cols to inspect)")
        print("  • Check --edge direction (spec: sample on falling edge)")
        print("  • Confirm capture covers at least 2 full frames "
              "(2 × 1584 bits = 99 μs at 32 MHz)")
        return

    # ------------------------------------------------------------------
    # Assemble output CSV
    #
    # Each output row = one decoded frame from one lane.
    # Columns match analysis/sine-all-5_decoded.csv:
    #   frame_time, D1_adc1_amp1, D1_adc2_amp1, ..., D8_adc4_amp16
    #
    # For a 4-lane capture all lanes are merged per-frame by cycle_cnt.
    # For single-lane captures only the active D-channels are populated.
    # ------------------------------------------------------------------
    out_path = Path(args.output) if args.output else in_path.with_suffix("").with_suffix("") \
        .parent / (in_path.stem + "_decoded.csv")

    # Build per-frame records indexed by cycle_cnt
    frame_records = {}  # cycle_cnt → {col: value}
    frame_times   = {}  # cycle_cnt → timestamp

    for ln in lane_nums:
        for fr in decoders[ln].frames:
            cyc = fr['cycle_cnt']
            if cyc not in frame_records:
                frame_records[cyc] = {}
                frame_times[cyc]   = fr['time']
            for w in range(3, 131):
                col = col_name(ln, w)
                val = fr['data'][w - 3]
                frame_records[cyc][col] = f"0x{val:03X}" if args.raw else str(val)

    # Sort by cycle counter (wraps at 4096, so use modular sort)
    sorted_cycles = sorted(frame_records.keys(),
                           key=lambda c: (c - min(frame_records.keys())) & 0xFFF)

    with open(out_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["frame_time", "cycle_cnt"] + ALL_COLS)
        for cyc in sorted_cycles:
            rec = frame_records[cyc]
            row = [f"{frame_times[cyc]:.9f}", cyc] + \
                  [rec.get(c, "") for c in ALL_COLS]
            writer.writerow(row)

    print(f"Wrote {len(sorted_cycles)} frame(s) → {out_path}")
    print(f"Columns: frame_time, cycle_cnt + {len(ALL_COLS)} ADC values "
          f"(D1..D8 × adc1..4 × amp1..16)")


if __name__ == "__main__":
    main()
