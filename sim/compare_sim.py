#!/usr/bin/env python3
"""
compare_sim.py  —  End-to-end efficacy test for the FPGA decoder simulation.

Flow:
  1. Reads sim/serial_sim.csv  (clock + 4 serial lanes dumped by tb_h5.v).
  2. Decodes each lane's bit stream with a gap-tolerant one-shot frame scanner:
       • Scans for SYNC word, accumulates 132 words, validates CRC-12.
       • No consecutive-frame requirement — handles zero-bit gaps between frames.
  3. Reads sim/expected_ch{0..7}.hex  (golden ADC values from extract_h5_stim.py).
  4. For each decoded frame, uses a sliding-window search over the golden data to
     find the correct group offset (frames are non-consecutive because the
     lane_framer skips ~24 groups during the 1584-cycle EMIT phase).
  5. Compares decoded ADC values against golden values at the matched offset.
  6. Prints per-frame and per-channel error metrics and PASS/FAIL verdict.
  7. Optionally plots the first frame's waveform per channel (--plot).

Usage:
  conda run -n base python3 sim/compare_sim.py
  conda run -n base python3 sim/compare_sim.py --plot
"""

import os
import sys
import csv
import argparse

try:
    import numpy as np
except ImportError:
    sys.exit("ERROR: numpy not found.  Run: conda install numpy")

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
SERIAL_CSV   = os.path.join(SCRIPT_DIR, 'serial_sim.csv')
EXP_HEX_FMT = os.path.join(SCRIPT_DIR, 'expected_ch{ch}.hex')

N_CH          = 8
N_ADC_PER_GP  = 4
N_GROUPS_FR   = 16
WORDS_PER_CH_PER_FRAME = N_GROUPS_FR * N_ADC_PER_GP   # 64

SYNC_WORDS  = [0xA35, 0xB46, 0xC57, 0xD68]
DATA_WORDS  = 128
FRAME_WORDS = DATA_WORDS + 4   # 132
CRC_POLY    = 0x80F
CRC_SEED    = 0x000

# =============================================================================
# CRC-12  (mirrors lane_framer's crc12_next exactly)
# =============================================================================
def crc12_update(crc, word):
    for i in range(11, -1, -1):
        if ((word >> i) & 1) ^ ((crc >> 11) & 1):
            crc = ((crc << 1) & 0xFFF) ^ CRC_POLY
        else:
            crc = (crc << 1) & 0xFFF
    return crc

# =============================================================================
# Gap-tolerant one-shot frame scanner
# Finds every complete, CRC-valid frame in a bit stream.
# Returns list of dicts: {frame_idx, data[128], crc_ok, lane}
# =============================================================================
def scan_frames(bits, lane):
    sync = SYNC_WORDS[lane]
    frames = []
    sreg   = 0
    i      = 0
    n      = len(bits)

    while i < n:
        # HUNT: shift bits until SYNC word found in sreg
        sreg = ((sreg << 1) | int(bits[i])) & 0xFFF
        i   += 1
        if sreg != sync:
            continue

        # SYNC found — accumulate remaining 131 words (12 bits each)
        words_left = FRAME_WORDS - 1   # we already have word 0 (SYNC)
        needed     = words_left * 12
        if i + needed > n:
            break   # not enough bits left

        frame_words = [sync]
        for w in range(words_left):
            word = 0
            for b in range(12):
                word = (word << 1) | int(bits[i])
                i   += 1
            frame_words.append(word)

        # Validate CRC-12 over words[3..130]
        crc = CRC_SEED
        for w in range(3, 131):
            crc = crc12_update(crc, frame_words[w])
        crc_ok = (crc == frame_words[131])

        if crc_ok:
            frames.append({
                'lane':      lane,
                'frame_idx': len(frames),
                'cycle_cnt': frame_words[2],
                'data':      frame_words[3:131],
            })
        # Whether CRC passed or not, resume scanning from current position

    return frames

# =============================================================================
# Load serial CSV and extract one bit stream per lane (sampled on falling edge)
# =============================================================================
def load_serial_csv():
    if not os.path.exists(SERIAL_CSV):
        sys.exit(f"ERROR: serial CSV not found: {SERIAL_CSV}\n"
                 "Run: make sim-h5  (or vvp sim_tb_h5)")

    print(f"Loading {SERIAL_CSV} ...", end=" ", flush=True)
    rows = []
    with open(SERIAL_CSV, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    print(f"{len(rows):,} samples")

    times = np.array([float(r['Time (s)']) for r in rows])  # noqa: F841
    clk   = np.array([int(r['la_clk'])    for r in rows], dtype=np.int8)
    lanes = [np.array([int(r[f'la_data_{l}']) for r in rows], dtype=np.uint8)
             for l in range(4)]

    # Falling edges: 1→0 transitions
    diff      = np.diff(clk)
    fall_idx  = np.where(diff < 0)[0] + 1
    print(f"  Found {len(fall_idx):,} falling clock edges")

    # Sample each lane at every falling edge
    bits = [lanes[l][fall_idx] for l in range(4)]
    return bits

# =============================================================================
# Load golden expected values from hex files
# =============================================================================
def load_expected():
    exp = []
    for ch in range(N_CH):
        path = EXP_HEX_FMT.format(ch=ch)
        if not os.path.exists(path):
            sys.exit(f"ERROR: {path} not found\n"
                     "Run: conda run -n base python3 sim/extract_h5_stim.py")
        vals = []
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line:
                    vals.append(int(line, 16))
        exp.append(np.array(vals, dtype=np.int32))
    return exp

# =============================================================================
# Sliding-window group-offset search
#
# The lane_framer emits a 1584-cycle serial packet between frames, during which
# the DUT continues producing data that gets discarded.  Each frame therefore
# maps to a non-consecutive block of groups in the golden data.
#
# For each decoded frame we search all candidate group offsets and return the
# one that gives zero mismatches against the golden expected values.
#
# Frame data layout:
#   data[0..127] = 128 words, interleaved even/odd channel
#   pair  = data_idx (0..127)
#   amp_f = pair // 2             (0..63)
#   group_local = amp_f // N_ADC  (0..15, group within this frame)
#   adc         = amp_f % N_ADC   (0..3)
#   is_odd      = pair % 2        (0=even ch, 1=odd ch)
#   ch          = 2*lane + is_odd
# =============================================================================
def find_frame_group_offset(frame, lane, exp):
    """Return (best_group_offset, min_error_count) for this frame."""
    total_golden_groups = len(exp[2 * lane]) // N_ADC_PER_GP   # 128
    search_end = total_golden_groups - N_GROUPS_FR + 1          # 113

    best_offset = 0
    best_errors = DATA_WORDS + 1  # worse than any real score

    for offset in range(search_end):
        errors = 0
        for pair, val in enumerate(frame['data']):
            amp_f       = pair // 2
            is_odd      = pair  % 2
            group_local = amp_f // N_ADC_PER_GP
            adc         = amp_f  % N_ADC_PER_GP
            ch          = 2 * lane + is_odd
            exp_idx     = (offset + group_local) * N_ADC_PER_GP + adc
            if exp_idx < len(exp[ch]) and exp[ch][exp_idx] != val:
                errors += 1
        if errors < best_errors:
            best_errors = errors
            best_offset = offset
        if errors == 0:
            break   # perfect match — no need to search further

    return best_offset, best_errors

# =============================================================================
# Build lookup: (ch, frame_idx, adc, group_in_frame) → value
# =============================================================================
def frames_to_lookup(frames_per_lane):
    lookup = {}
    for lane, frames in enumerate(frames_per_lane):
        for frame in frames:
            fi = frame['frame_idx']
            for pair, val in enumerate(frame['data']):
                amp_f  = pair // 2
                is_odd = pair  % 2
                group  = amp_f // N_ADC_PER_GP
                adc    = amp_f  % N_ADC_PER_GP
                ch     = 2 * lane + is_odd
                lookup[(ch, fi, adc, group)] = val
    return lookup

# =============================================================================
# Main
# =============================================================================
def main():
    parser = argparse.ArgumentParser(
        description="End-to-end FPGA decoder efficacy test")
    parser.add_argument('--plot', action='store_true',
                        help='Show per-channel waveform comparison plots')
    args = parser.parse_args()

    # -- Load serial data and decode ------------------------------------------
    lane_bits = load_serial_csv()

    print("Scanning for frames ...")
    frames_per_lane = []
    for l in range(4):
        frames = scan_frames(lane_bits[l], l)
        frames_per_lane.append(frames)
        status = f"{len(frames)} frame(s)"
        print(f"  Lane {l} (SYNC=0x{SYNC_WORDS[l]:03X}): {status}")

    total_frames = sum(len(f) for f in frames_per_lane)
    if total_frames == 0:
        sys.exit("ERROR: no frames decoded — check simulation output")

    # -- Load golden values ---------------------------------------------------
    print("\nLoading golden expected values ...")
    exp = load_expected()
    print(f"  {len(exp[0])} words/channel × {N_CH} channels")

    # -- Sliding-window search for group offsets per frame -------------------
    print("\nSearching for golden group offsets ...")
    # frame_offsets[(lane, fi)] = (group_offset, match_errors)
    frame_offsets = {}
    for lane, frames in enumerate(frames_per_lane):
        for frame in frames:
            fi = frame['frame_idx']
            offset, errors = find_frame_group_offset(frame, lane, exp)
            frame_offsets[(lane, fi)] = (offset, errors)
            status = "exact match" if errors == 0 else f"{errors} mismatches"
            print(f"  Lane {lane} Frame {fi}: group_offset={offset:3d}  ({status})")

    # -- Build lookup and compare using correct offsets ----------------------
    lookup = frames_to_lookup(frames_per_lane)

    print("\nComparing decoded values against golden ...")
    ch_results = {}
    all_errs   = []

    for ch in range(N_CH):
        lane   = ch // 2
        ch_errs, ch_got, ch_ref = [], [], []

        for frame in frames_per_lane[lane]:
            fi = frame['frame_idx']
            group_offset, _ = frame_offsets[(lane, fi)]

            for group in range(N_GROUPS_FR):
                for adc in range(N_ADC_PER_GP):
                    exp_idx = (group_offset + group) * N_ADC_PER_GP + adc
                    if exp_idx >= len(exp[ch]):
                        break
                    key = (ch, fi, adc, group)
                    if key not in lookup:
                        continue
                    got = lookup[key]
                    ref = int(exp[ch][exp_idx])
                    ch_errs.append(got - ref)
                    ch_got.append(got)
                    ch_ref.append(ref)

        if ch_errs:
            arr = np.array(ch_errs)
            ch_results[ch] = {
                'n':    len(arr),
                'max':  int(np.max(np.abs(arr))),
                'rmse': float(np.sqrt(np.mean(arr**2))),
                'mean': float(np.mean(arr)),
                'got':  np.array(ch_got),
                'ref':  np.array(ch_ref),
            }
            all_errs.extend(ch_errs)
        else:
            ch_results[ch] = None

    # -- Report ---------------------------------------------------------------
    print()
    print("=" * 65)
    print(f"  END-TO-END EFFICACY  —  {total_frames} frame(s) decoded across 4 lanes")
    print("=" * 65)
    print(f"  {'Ch':>3}  {'N':>6}  {'Max|err|':>9}  {'RMSE':>8}  {'Mean':>8}  Status")
    print(f"  {'-'*3}  {'-'*6}  {'-'*9}  {'-'*8}  {'-'*8}  {'------'}")

    any_fail = False
    for ch in range(N_CH):
        r = ch_results[ch]
        if r is None:
            print(f"  D{ch+1:>2}  {'—':>6}  {'—':>9}  {'—':>8}  {'—':>8}  NO DATA")
            continue
        ok = r['max'] == 0
        if not ok:
            any_fail = True
        status = "PASS" if ok else f"FAIL (max err={r['max']} LSB)"
        print(f"  D{ch+1:>2}  {r['n']:>6}  {r['max']:>9}  {r['rmse']:>8.4f}  {r['mean']:>8.4f}  {status}")

    if all_errs:
        arr = np.array(all_errs)
        print()
        print(f"  Total: {len(arr)} values compared")
        print(f"  Global RMSE    = {np.sqrt(np.mean(arr**2)):.4f} LSB")
        print(f"  Max |error|    = {int(np.max(np.abs(arr)))} LSB")
        print()
        verdict = ("PASS — decoder output exactly matches golden H5 reference"
                   if not any_fail else
                   "FAIL — decoder output differs from golden reference")
        print(f"  VERDICT: {verdict}")
    else:
        print("\n  WARNING: no values could be compared")

    print("=" * 65)

    # -- Optional plots -------------------------------------------------------
    if args.plot and ch_results:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("\nWARNING: matplotlib not found — skipping plots")
            return

        fig, axes = plt.subplots(4, 2, figsize=(14, 10))
        fig.suptitle("Sim decoder output vs golden H5 reference\n(per channel, first matched frame)", fontsize=12)

        for ch in range(N_CH):
            r  = ch_results[ch]
            ax = axes[ch // 2][ch % 2]
            ax.set_title(f"D{ch+1}")
            if r is None:
                ax.text(0.5, 0.5, 'No data', ha='center', va='center',
                        transform=ax.transAxes)
                continue
            n_plot = min(len(r['ref']), WORDS_PER_CH_PER_FRAME)
            ax.plot(r['ref'][:n_plot], 'b-', linewidth=0.8, label='expected (golden)')
            ax.plot(r['got'][:n_plot], 'r--', linewidth=0.8, alpha=0.8, label='decoded (sim)')
            ax.set_ylabel('ADC count (12-bit)')
            ax.legend(fontsize=7)
            label = 'EXACT MATCH' if r['max'] == 0 else f'max err = {r["max"]} LSB'
            color = 'green' if r['max'] == 0 else 'red'
            ax.text(0.98, 0.95, label, ha='right', va='top',
                    transform=ax.transAxes, color=color, fontsize=8)

        plt.tight_layout()
        plot_path = os.path.join(SCRIPT_DIR, 'compare_sim.png')
        plt.savefig(plot_path, dpi=120)
        print(f"\n  Plot saved → {plot_path}")
        plt.show()


if __name__ == '__main__':
    main()
