#!/usr/bin/env python3
"""
plot_comparison.py  —  Visualize sim_decoded.csv vs sine-all-5_decoded.csv

Loads both CSVs and produces two figures:
  Fig 1: Per-channel waveform comparison (D1..D8), ADC1 amplitude across groups.
          Reference shown in full; sim overlaid at its best-matching position.
  Fig 2: Correlation scatter — sim value vs reference value for each channel.

Usage:
  conda run -n base python3 sim/plot_comparison.py
  conda run -n base python3 sim/plot_comparison.py --save
"""

import os, sys, csv, argparse
import numpy as np

try:
    import matplotlib
    matplotlib.use('Agg')          # non-interactive backend; remove for GUI
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
except ImportError:
    sys.exit("ERROR: matplotlib not found — conda install matplotlib")

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
SIM_CSV     = os.path.join(SCRIPT_DIR, 'sim_decoded.csv')
REF_CSV     = os.path.join(SCRIPT_DIR, '..', 'sine-all-5_decoded.csv')
OUT_FIG1    = os.path.join(SCRIPT_DIR, 'compare_waveform.png')
OUT_FIG2    = os.path.join(SCRIPT_DIR, 'compare_scatter.png')

N_ADC      = 4
N_GROUPS   = 16
N_D        = 8     # D1..D8

# =============================================================================
# Load a decoded CSV → list of rows, each row is dict col→int
# =============================================================================
def load_csv(path):
    if not os.path.exists(path):
        sys.exit(f"ERROR: {path} not found")
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
# Extract per-channel, per-adc time series from a list of rows.
# Returns dict: (d, adc) → np.array of length n_rows * N_GROUPS
# d=1..8, adc=1..4
# Values ordered: [row0_amp1..amp16, row1_amp1..amp16, ...]
# =============================================================================
def extract_series(rows, d, adc):
    vals = []
    for row in rows:
        for g in range(1, N_GROUPS + 1):
            col = f'D{d}_adc{adc}_amp{g}'
            vals.append(row.get(col, 0))
    return np.array(vals, dtype=np.float32)

# =============================================================================
# Find the reference row index where the sim row best aligns
# (minimise sum of squared differences for D1 ADC1 as a proxy)
# =============================================================================
def find_best_ref_offset(sim_rows, ref_rows):
    """Return list of best ref row indices for each sim row."""
    offsets = []
    for si, srow in enumerate(sim_rows):
        best_ri  = 0
        best_err = float('inf')
        for ri, rrow in enumerate(ref_rows):
            err = sum(
                (srow.get(f'D{d}_adc{a}_amp{g}', 0) -
                 rrow.get(f'D{d}_adc{a}_amp{g}', 0))**2
                for d in range(1, 9)
                for a in range(1, 5)
                for g in range(1, 17)
            )
            if err < best_err:
                best_err = err
                best_ri  = ri
        offsets.append(best_ri)
    return offsets

# =============================================================================
# Figure 1: waveform comparison — full reference + sim overlay
# =============================================================================
def plot_waveforms(sim_rows, ref_rows, best_offsets, save):
    fig, axes = plt.subplots(N_D, 1, figsize=(16, 14), sharex=False)
    fig.suptitle(
        "Decoder output vs hardware reference — ADC1, all groups across frames\n"
        f"Sim: {len(sim_rows)} frame(s)   Reference: {len(ref_rows)} frame(s)",
        fontsize=12
    )

    for idx, d in enumerate(range(1, N_D + 1)):
        ax = axes[idx]
        adc = 1   # show ADC1 for each channel

        # Full reference series
        ref_series = extract_series(ref_rows, d, adc)
        ref_x = np.arange(len(ref_series))
        ax.plot(ref_x, ref_series, color='steelblue', linewidth=0.6,
                alpha=0.7, label='Reference (full)')

        # Sim series — plot at best-matching position in reference timeline
        for si, (srow, ri) in enumerate(zip(sim_rows, best_offsets)):
            sim_vals = [srow.get(f'D{d}_adc{adc}_amp{g}', 0)
                        for g in range(1, N_GROUPS + 1)]
            x_start = ri * N_GROUPS
            sim_x   = np.arange(x_start, x_start + N_GROUPS)
            label   = f'Sim frame {si} (→ref row {ri})' if idx == 0 else '_nolegend_'
            ax.plot(sim_x, sim_vals, 'o-', color='crimson',
                    linewidth=1.2, markersize=3, alpha=0.9, label=label)

        ax.set_ylabel(f'D{d} ADC1\n(12-bit count)', fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(True, alpha=0.3)
        if idx == 0:
            ax.legend(fontsize=7, loc='upper right')

    axes[-1].set_xlabel('Group index (across all frames)', fontsize=9)
    plt.tight_layout()

    if save:
        plt.savefig(OUT_FIG1, dpi=120)
        print(f"  Saved → {OUT_FIG1}")
    else:
        plt.show()
    plt.close()

# =============================================================================
# Figure 2: scatter — sim value vs best-match reference value per channel
# =============================================================================
def plot_scatter(sim_rows, ref_rows, best_offsets, save):
    fig, axes = plt.subplots(2, 4, figsize=(16, 8))
    fig.suptitle(
        "Sim decoded vs hardware reference — scatter per channel\n"
        "(all ADCs, all groups, all sim frames vs best-match reference rows)",
        fontsize=11
    )

    for idx, d in enumerate(range(1, N_D + 1)):
        ax = axes[idx // 4][idx % 4]
        sim_vals, ref_vals = [], []

        for si, (srow, ri) in enumerate(zip(sim_rows, best_offsets)):
            rrow = ref_rows[ri]
            for a in range(1, N_ADC + 1):
                for g in range(1, N_GROUPS + 1):
                    col = f'D{d}_adc{a}_amp{g}'
                    sim_vals.append(srow.get(col, 0))
                    ref_vals.append(rrow.get(col, 0))

        sim_arr = np.array(sim_vals)
        ref_arr = np.array(ref_vals)

        ax.scatter(ref_arr, sim_arr, s=4, alpha=0.4, color='steelblue')

        # Identity line
        lo = min(ref_arr.min(), sim_arr.min())
        hi = max(ref_arr.max(), sim_arr.max())
        ax.plot([lo, hi], [lo, hi], 'r--', linewidth=0.8, alpha=0.6,
                label='y=x (perfect)')

        # Stats
        rmse = float(np.sqrt(np.mean((sim_arr - ref_arr)**2)))
        bias = float(np.mean(sim_arr - ref_arr))
        ax.set_title(f'D{d}  RMSE={rmse:.0f}  bias={bias:+.0f}', fontsize=8)
        ax.set_xlabel('Reference value (LSB)', fontsize=7)
        ax.set_ylabel('Sim value (LSB)', fontsize=7)
        ax.tick_params(labelsize=7)
        ax.legend(fontsize=6)

    plt.tight_layout()

    if save:
        plt.savefig(OUT_FIG2, dpi=120)
        print(f"  Saved → {OUT_FIG2}")
    else:
        plt.show()
    plt.close()

# =============================================================================
# Main
# =============================================================================
def main():
    parser = argparse.ArgumentParser(
        description="Visualize sim_decoded.csv vs sine-all-5_decoded.csv")
    parser.add_argument('--save', action='store_true',
                        help='Save PNGs instead of showing interactive window')
    args = parser.parse_args()

    print(f"Loading sim:  {SIM_CSV}")
    sim_rows = load_csv(SIM_CSV)
    print(f"  {len(sim_rows)} frame(s)")

    ref_path = os.path.normpath(REF_CSV)
    print(f"Loading ref:  {ref_path}")
    ref_rows = load_csv(ref_path)
    print(f"  {len(ref_rows)} frame(s)")

    print("Finding best reference alignment ...")
    best_offsets = find_best_ref_offset(sim_rows, ref_rows)
    for si, ri in enumerate(best_offsets):
        print(f"  Sim frame {si} → reference row {ri}")

    print("\nPlotting waveforms ...")
    plot_waveforms(sim_rows, ref_rows, best_offsets, args.save)

    print("Plotting scatter ...")
    plot_scatter(sim_rows, ref_rows, best_offsets, args.save)

    print("Done.")


if __name__ == '__main__':
    main()
