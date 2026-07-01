#!/usr/bin/env python3
"""
plot_signal.py  —  Visualize all 512 waveforms from sim_decoded.csv.

Structure:
  512 waveforms = 8 D-channels × 4 ADCs × 16 groups (range bins)
  Each waveform gets ONE new sample per frame (every ~80 µs in sim).

Display: one heatmap per D-channel (8 subplots).
  y-axis : group index 1..16  (range bin)
  x-axis : frame time (µs, from frame_time column)
  colour : ADC amplitude averaged over the 4 ADC chips

Usage:
    conda run -n base python3 sim/plot_signal.py [--save]
"""

import os, sys, csv, argparse
import numpy as np

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("ERROR: matplotlib not found")

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_CSV    = os.path.join(SCRIPT_DIR, 'sim_decoded.csv')
OUT_PNG    = os.path.join(SCRIPT_DIR, 'signal_heatmap.png')

N_ADC    = 4
N_GROUPS = 16
ADC_MIN  = 512    # reject obviously corrupted words
ADC_MAX  = 3584


def load_csv(path):
    rows = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            try:
                float(row['frame_time'])
            except (ValueError, KeyError):
                continue
            rows.append(row)
    return rows


def build_heatmap(rows, d):
    """Return (frame_times_us, grid) where grid[g, f] = mean ADC value
    for D-channel d, group g+1, frame f. NaN for missing/corrupt."""
    n_frames = len(rows)
    grid = np.full((N_GROUPS, n_frames), np.nan)
    times = np.array([float(r['frame_time']) * 1e6 for r in rows])

    for fi, row in enumerate(rows):
        for g in range(N_GROUPS):
            vals = []
            for a in range(1, N_ADC + 1):
                col = f'D{d}_adc{a}_amp{g+1}'
                v = row.get(col, '').strip()
                if v:
                    iv = int(v)
                    if ADC_MIN <= iv <= ADC_MAX:
                        vals.append(iv)
            if vals:
                grid[g, fi] = np.mean(vals)

    return times, grid


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--save', action='store_true')
    args = parser.parse_args()

    if not os.path.exists(SIM_CSV):
        sys.exit(f"ERROR: {SIM_CSV} not found")

    rows = load_csv(SIM_CSV)
    n_frames = len(rows)
    print(f"Loaded {n_frames} frame(s)  →  {n_frames * N_GROUPS * N_ADC * 8} total ADC readings")

    fig, axes = plt.subplots(8, 1, figsize=(14, 16), sharex=False)
    fig.suptitle(
        f'512 waveforms — sim_decoded.csv  ({n_frames} frames)\n'
        f'Each cell = mean of 4 ADC chips | y = range bin (group) | x = frame time',
        fontsize=11
    )

    for idx, d in enumerate(range(1, 9)):
        ax = axes[idx]
        times, grid = build_heatmap(rows, d)

        vmin, vmax = np.nanpercentile(grid, 2), np.nanpercentile(grid, 98)
        im = ax.imshow(
            grid,
            aspect='auto',
            origin='lower',
            extent=[times[0], times[-1], 0.5, N_GROUPS + 0.5],
            vmin=vmin, vmax=vmax,
            cmap='viridis',
            interpolation='nearest'
        )
        fig.colorbar(im, ax=ax, fraction=0.02, pad=0.01, label='LSB')
        ax.set_ylabel(f'D{d}\ngroup', fontsize=8)
        ax.set_yticks([1, 4, 8, 12, 16])
        ax.tick_params(labelsize=7)

    axes[-1].set_xlabel('Frame time (µs)', fontsize=9)
    plt.tight_layout()

    if args.save:
        plt.savefig(OUT_PNG, dpi=130)
        print(f"Saved → {OUT_PNG}")
    else:
        plt.show()
    plt.close()


if __name__ == '__main__':
    main()
