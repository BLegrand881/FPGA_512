#!/usr/bin/env python3
"""
compare_sim.py  —  End-to-end FPGA decoder efficacy pipeline.

Pipeline:
  1. Reads sim/serial_sim.csv  (clock + 4 serial lanes from tb_h5.v).
  2. Decodes each lane with a gap-tolerant CRC-12 frame scanner
     (handles zero-bit gaps between frames that break receiver.py).
  3. Writes sim/sim_decoded.csv in the standard receiver.py output format:
       frame_time, D1_adc1_amp1, ..., D8_adc4_amp16   (512 values/row)
  4. Loads the hardware reference sine-all-5_decoded.csv.
  5. For each sim frame, finds the best-matching reference row (sliding window)
     and reports per-channel RMSE and match statistics.

Usage:
  conda run -n base python3 sim/compare_sim.py
  conda run -n base python3 sim/compare_sim.py --plot
"""

import os, sys, csv, argparse
import numpy as np

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
SERIAL_CSV  = os.path.join(SCRIPT_DIR, 'serial_sim.csv')
SIM_OUT_CSV = os.path.join(SCRIPT_DIR, 'sim_decoded.csv')
REF_CSV     = os.path.join(SCRIPT_DIR, '..', 'sine-all-5_decoded.csv')

N_CH         = 8
N_ADC        = 4
N_GROUPS_FR  = 16
DATA_WORDS   = 128   # per lane per frame

SYNC_WORDS   = [0xA35, 0xB46, 0xC57, 0xD68]
FRAME_WORDS  = DATA_WORDS + 4   # SYNC + LANE_ID + CYCLE_CNT + data + CRC
CRC_POLY     = 0x80F
CRC_SEED     = 0x000

# Output column order — matches sine-all-5_decoded.csv exactly
OUT_COLS = ['frame_time'] + [
    f'D{d}_adc{a}_amp{g}'
    for d in range(1, 9)
    for a in range(1, 5)
    for g in range(1, 17)
]

# =============================================================================
# CRC-12
# =============================================================================
def crc12_update(crc, word):
    for i in range(11, -1, -1):
        if ((word >> i) & 1) ^ ((crc >> 11) & 1):
            crc = ((crc << 1) & 0xFFF) ^ CRC_POLY
        else:
            crc = (crc << 1) & 0xFFF
    return crc

# =============================================================================
# Load serial CSV → bits + timestamps per lane (sampled on falling edges)
# =============================================================================
def load_serial_csv():
    if not os.path.exists(SERIAL_CSV):
        sys.exit(f"ERROR: {SERIAL_CSV} not found — run: make sim-h5")

    print(f"Loading {SERIAL_CSV} ...", end=" ", flush=True)
    rows = []
    with open(SERIAL_CSV, newline='') as f:
        for row in csv.DictReader(f):
            rows.append(row)
    print(f"{len(rows):,} samples")

    times = np.array([float(r['Time (s)']) for r in rows])
    clk   = np.array([int(r['la_clk'])    for r in rows], dtype=np.int8)
    lanes = [np.array([int(r[f'la_data_{l}']) for r in rows], dtype=np.uint8)
             for l in range(4)]

    fall_idx  = np.where(np.diff(clk) < 0)[0] + 1
    fall_times = times[fall_idx]
    print(f"  {len(fall_idx):,} falling clock edges")

    bits       = [lanes[l][fall_idx] for l in range(4)]
    return bits, fall_times

# =============================================================================
# Gap-tolerant one-shot frame scanner
# Returns list of dicts: {frame_idx, data[128], time}
# =============================================================================
def scan_frames(bits, times, lane):
    sync   = SYNC_WORDS[lane]
    frames = []
    sreg   = 0
    i      = 0
    n      = len(bits)

    while i < n:
        sreg = ((sreg << 1) | int(bits[i])) & 0xFFF
        i   += 1
        if sreg != sync:
            continue

        words_left = FRAME_WORDS - 1
        needed     = words_left * 12
        if i + needed > n:
            break

        frame_time  = float(times[i - 1]) if i > 0 else 0.0
        frame_words = [sync]
        for _ in range(words_left):
            word = 0
            for _ in range(12):
                word = (word << 1) | int(bits[i])
                i   += 1
            frame_words.append(word)

        crc = CRC_SEED
        for w in range(3, 131):
            crc = crc12_update(crc, frame_words[w])

        if crc == frame_words[131]:
            frames.append({
                'lane':      lane,
                'frame_idx': len(frames),
                'time':      frame_time,
                'data':      frame_words[3:131],
            })

    return frames

# =============================================================================
# Write sim_decoded.csv — same format as receiver.py / sine-all-5_decoded.csv
#
# Each row = one sim frame index (all 4 lanes merged).
# Pair → column mapping (same as receiver.py col_name()):
#   pair k: amp_f=k//2, is_odd=k%2, group=amp_f//4, adc=amp_f%4
#   col    = D{2*lane+is_odd+1}_adc{adc+1}_amp{group+1}
# =============================================================================
def write_sim_decoded_csv(frames_per_lane, path):
    n_frames = max(len(f) for f in frames_per_lane)

    rows = []
    for fi in range(n_frames):
        row = {col: '' for col in OUT_COLS}
        # frame_time from lane 0
        if fi < len(frames_per_lane[0]):
            row['frame_time'] = f"{frames_per_lane[0][fi]['time']:.12e}"
        else:
            row['frame_time'] = '0'

        for lane in range(4):
            if fi >= len(frames_per_lane[lane]):
                continue
            for pair, val in enumerate(frames_per_lane[lane][fi]['data']):
                amp_f   = pair // 2
                is_odd  = pair  % 2
                group   = amp_f // N_ADC
                adc     = amp_f  % N_ADC
                d       = 2 * lane + is_odd + 1
                col     = f'D{d}_adc{adc+1}_amp{group+1}'
                row[col] = str(val)
        rows.append(row)

    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=OUT_COLS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    print(f"  Wrote {len(rows)} frame(s) → {path}")

# =============================================================================
# Load reference CSV (sine-all-5_decoded.csv or sim_decoded.csv)
# Returns list of {col→int} dicts (data rows only, skipping RMS summary rows)
# =============================================================================
def load_decoded_csv(path):
    rows = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            try:
                float(row['frame_time'])
            except (ValueError, KeyError):
                continue
            rows.append({k: int(v) for k, v in row.items()
                         if k != 'frame_time' and v.strip() != ''})
    return rows

# =============================================================================
# Compare sim_decoded.csv rows against reference rows
# For each sim row, search all ref rows for the best-matching one.
# =============================================================================
def compare_decoded(sim_rows, ref_rows):
    results = []   # (sim_idx, best_ref_idx, errors, total)
    for si, srow in enumerate(sim_rows):
        cols    = [c for c in srow if c in (ref_rows[0] if ref_rows else {})]
        best_ri = 0
        best_err = len(cols) + 1
        for ri, rrow in enumerate(ref_rows):
            err = sum(1 for c in cols if srow.get(c) != rrow.get(c))
            if err < best_err:
                best_err = err
                best_ri  = ri
            if err == 0:
                break
        results.append((si, best_ri, best_err, len(cols)))
    return results

# =============================================================================
# Per-channel RMSE between sim_decoded and its best-matching reference row
# =============================================================================
def channel_stats(sim_rows, ref_rows, match_results):
    ch_errs = {d: [] for d in range(1, 9)}
    for si, ri, _, _ in match_results:
        if ri >= len(ref_rows):
            continue
        srow = sim_rows[si]
        rrow = ref_rows[ri]
        for d in range(1, 9):
            for a in range(1, 5):
                for g in range(1, 17):
                    col = f'D{d}_adc{a}_amp{g}'
                    if col in srow and col in rrow:
                        ch_errs[d].append(srow[col] - rrow[col])
    stats = {}
    for d, errs in ch_errs.items():
        if errs:
            arr = np.array(errs)
            stats[d] = {
                'n':    len(arr),
                'rmse': float(np.sqrt(np.mean(arr**2))),
                'max':  int(np.max(np.abs(arr))),
                'mean': float(np.mean(arr)),
            }
    return stats

# =============================================================================
# Main
# =============================================================================
def main():
    parser = argparse.ArgumentParser(description="FPGA decoder end-to-end efficacy")
    parser.add_argument('--plot', action='store_true',
                        help='Plot sim vs reference waveforms')
    args = parser.parse_args()

    # 1. Load and decode serial simulation output
    lane_bits, fall_times = load_serial_csv()

    print("Scanning for frames ...")
    frames_per_lane = []
    for l in range(4):
        frames = scan_frames(lane_bits[l], fall_times, l)
        frames_per_lane.append(frames)
        print(f"  Lane {l} (SYNC=0x{SYNC_WORDS[l]:03X}): {len(frames)} frame(s)")

    total_frames = sum(len(f) for f in frames_per_lane)
    if total_frames == 0:
        sys.exit("ERROR: no frames decoded — run: make sim-h5")

    # 2. Write sim_decoded.csv in standard receiver output format
    print("\nWriting decoded output ...")
    write_sim_decoded_csv(frames_per_lane, SIM_OUT_CSV)

    # 3. Load both CSVs for comparison
    ref_path = os.path.normpath(REF_CSV)
    if not os.path.exists(ref_path):
        print(f"\nNo reference CSV found at {ref_path} — skipping comparison")
        return

    print(f"\nLoading reference: {ref_path}")
    sim_rows = load_decoded_csv(SIM_OUT_CSV)
    ref_rows = load_decoded_csv(ref_path)
    print(f"  sim_decoded.csv : {len(sim_rows)} frame(s)")
    print(f"  reference CSV   : {len(ref_rows)} frame(s)")

    # 4. Find best-matching reference row for each sim frame
    print("\nMatching sim frames against reference ...")
    match_results = compare_decoded(sim_rows, ref_rows)

    print()
    print("=" * 68)
    print(f"  FRAME MATCH RESULTS")
    print("=" * 68)
    print(f"  {'SimRow':>6}  {'RefRow':>6}  {'Errs':>6}  {'Total':>6}  {'Match%':>7}  Status")
    print(f"  {'-'*6}  {'-'*6}  {'-'*6}  {'-'*6}  {'-'*7}  ------")
    all_pass = True
    for si, ri, err, total in match_results:
        pct = 100.0 * (total - err) / total if total else 0
        ok  = (err == 0)
        if not ok:
            all_pass = False
        status = "PASS" if ok else f"FAIL ({err} mismatches)"
        print(f"  {si:>6}  {ri:>6}  {err:>6}  {total:>6}  {pct:>6.1f}%  {status}")

    # 5. Per-channel stats
    stats = channel_stats(sim_rows, ref_rows, match_results)

    print()
    print("=" * 68)
    print(f"  PER-CHANNEL COMPARISON  (sim vs best-matching reference row)")
    print("=" * 68)
    print(f"  {'Ch':>3}  {'N':>5}  {'RMSE (LSB)':>11}  {'Max|err|':>9}  {'Mean err':>9}")
    print(f"  {'-'*3}  {'-'*5}  {'-'*11}  {'-'*9}  {'-'*9}")
    global_errs = []
    for d in range(1, 9):
        s = stats.get(d)
        if s:
            print(f"  D{d:>2}  {s['n']:>5}  {s['rmse']:>11.2f}  {s['max']:>9}  {s['mean']:>9.2f}")
            global_errs.extend([0]*s['n'])   # placeholder — detailed errs in stats
        else:
            print(f"  D{d:>2}  {'—':>5}  {'—':>11}  {'—':>9}  {'—':>9}")

    if stats:
        all_rmse = np.sqrt(np.mean([s['rmse']**2 for s in stats.values()]))
        all_max  = max(s['max'] for s in stats.values())
        print(f"\n  Global RMSE  = {all_rmse:.2f} LSB")
        print(f"  Global max   = {all_max} LSB")

    verdict = "PASS" if all_pass else "FAIL — values differ from reference"
    print(f"\n  VERDICT: {verdict}")
    print("=" * 68)

    # 6. Optional plots
    if args.plot:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("WARNING: matplotlib not found — skipping plots")
            return

        if not sim_rows or not ref_rows:
            return

        si, ri, _, _ = match_results[0]
        srow = sim_rows[si]
        rrow = ref_rows[ri]

        fig, axes = plt.subplots(4, 2, figsize=(14, 10))
        fig.suptitle(f"sim_decoded (row {si}) vs reference (row {ri})\n"
                     "per channel, 4 ADCs × 16 groups = 64 values", fontsize=11)

        for d in range(1, 9):
            ax = axes[(d-1)//2][(d-1)%2]
            ax.set_title(f"D{d}")
            sim_vals = [srow.get(f'D{d}_adc{a}_amp{g}', 0)
                        for a in range(1,5) for g in range(1,17)]
            ref_vals = [rrow.get(f'D{d}_adc{a}_amp{g}', 0)
                        for a in range(1,5) for g in range(1,17)]
            ax.plot(ref_vals, 'b-',  linewidth=0.8, label='reference')
            ax.plot(sim_vals, 'r--', linewidth=0.8, alpha=0.8, label='sim decoded')
            ax.set_ylabel('ADC count')
            ax.legend(fontsize=7)

        plt.tight_layout()
        plot_path = os.path.join(SCRIPT_DIR, 'compare_sim.png')
        plt.savefig(plot_path, dpi=120)
        print(f"\n  Plot saved → {plot_path}")
        plt.show()


if __name__ == '__main__':
    main()
