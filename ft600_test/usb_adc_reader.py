#!/usr/bin/env python3
"""
usb_adc_reader.py — Decode packed serial lane data from a raw binary dump.

The FPGA packs 4 lane_framer serial outputs into 16-bit USB words:
  [15:12] = time t0: {lane3, lane2, lane1, lane0}
  [11:8]  = time t1: {lane3, lane2, lane1, lane0}
  [7:4]   = time t2: {lane3, lane2, lane1, lane0}
  [3:0]   = time t3: {lane3, lane2, lane1, lane0}

Workflow:
  1. Capture:  ./dump_raw 10 raw.bin      (C program, uses D3XX)
  2. Decode:   python usb_adc_reader.py raw.bin -o decoded.csv

Usage:
    python usb_adc_reader.py raw.bin
    python usb_adc_reader.py raw.bin -o decoded.csv
    python usb_adc_reader.py raw.bin --raw   # hex output
"""

import argparse
import struct
import sys
import csv
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "receiver"))
from receiver import LaneDecoder, col_name, ALL_COLS, SYNC_WORDS, STATE_NAMES, crc12_update, CRC_SEED


class UsbLaneDecoder(LaneDecoder):
    """LaneDecoder that emits frames after first CRC validation.

    The USB bit stream has idle gaps between frames (lane_framer COLLECT phase),
    so PRE_LOCK always fails. This subclass emits from ALIGN on CRC pass and
    returns to HUNT for the next frame.
    """
    def _align(self, word, ts):
        w = self.wptr
        if w == 1:
            if word != self.exp_lid:
                self.n_lane_id += 1
                self._hunt()
            else:
                self.wptr += 1
        elif w == 2:
            self.rx_cnt  = word
            self.exp_cnt = (word + 1) & 0xFFF
            self.wptr   += 1
        elif 3 <= w <= 130:
            self.dbuf[w - 3] = word
            self.crc_acc     = crc12_update(self.crc_acc, word)
            self.wptr       += 1
        elif w == 131:
            if word != self.crc_acc:
                self.n_crc += 1
                self._hunt()
            else:
                self._emit_frame()
                self.crc_acc = CRC_SEED
                self._hunt()  # back to HUNT — idle gap before next frame


def main():
    parser = argparse.ArgumentParser(
        description="Decode packed serial lane data from raw binary USB dump.")
    parser.add_argument("input", help="Raw binary file from dump_raw")
    parser.add_argument("-o", "--output", default=None,
                        help="Output CSV file (default: <input>_decoded.csv)")
    parser.add_argument("--raw", action="store_true",
                        help="Output hex ADC values instead of decimal")
    args = parser.parse_args()

    in_path = Path(args.input)
    if not in_path.exists():
        sys.exit(f"ERROR: file not found: {in_path}")

    data = in_path.read_bytes()
    n_bytes = len(data) & ~1
    n_words = n_bytes // 2
    print(f"Loaded {in_path}: {len(data):,} bytes ({n_words:,} words)")

    if n_words == 0:
        sys.exit("ERROR: empty file")

    words = struct.unpack(f"<{n_words}H", data[:n_bytes])

    # Count non-zero words for sanity check
    nonzero = sum(1 for w in words if w != 0)
    print(f"  Non-zero words: {nonzero:,} / {n_words:,} ({100*nonzero/n_words:.1f}%)")

    # Decode
    decoders = [UsbLaneDecoder(i) for i in range(4)]

    print("Decoding ...", end=" ", flush=True)
    bit_count = 0
    for word in words:
        for shift in (12, 8, 4, 0):
            nibble = (word >> shift) & 0xF
            for lane in range(4):
                bit = (nibble >> lane) & 1
                decoders[lane].feed_bit(bit, timestamp=bit_count / 16e6)
            bit_count += 1
    print("done")

    # Results
    total_decoded = 0
    print(f"\n{'='*60}")
    for d in decoders:
        ln = d.lane
        state = STATE_NAMES[d.state]
        print(f"\n  Lane {ln} (SYNC=0x{SYNC_WORDS[ln]:03X}, "
              f"D{2*ln+1}/D{2*ln+2}, state: {state})")
        print(f"    Decoded:       {d.n_decoded}")
        print(f"    SYNC losses:   {d.n_sync_loss}")
        print(f"    LANE_ID errs:  {d.n_lane_id}")
        print(f"    CRC errors:    {d.n_crc}")
        print(f"    Seq gaps:      {d.n_seq}")
        total_decoded += d.n_decoded

    print()

    if total_decoded == 0:
        print("WARNING: no frames decoded.")
        print("  Check bit packing order or inspect with: xxd raw.bin | head -20")
        sys.exit(1)

    # Assemble CSV
    out_path = Path(args.output) if args.output else in_path.with_suffix("").with_name(
        in_path.stem + "_decoded.csv")

    frame_records = {}
    frame_times = {}

    for d in decoders:
        for fr in d.frames:
            cyc = fr['cycle_cnt']
            if cyc not in frame_records:
                frame_records[cyc] = {}
                frame_times[cyc] = fr['time']
            for w in range(3, 131):
                col = col_name(d.lane, w)
                val = fr['data'][w - 3]
                frame_records[cyc][col] = f"0x{val:03X}" if args.raw else str(val)

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

    total_errors = sum(dd.n_crc + dd.n_sync_loss + dd.n_lane_id for dd in decoders)
    if total_errors == 0:
        print("\n  PASS — all frames decoded with zero errors")
    else:
        print(f"\n  WARN — {total_errors} total error(s) across all lanes")

    sys.exit(0 if total_errors == 0 else 1)


if __name__ == "__main__":
    main()
