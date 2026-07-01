#!/usr/bin/env python3
"""
plot_raw.py  —  Plot 8 randomly selected waveforms from sim_decoded.csv.

Each of the 512 independent waveforms is identified by (D-channel, ADC, group).
Each waveform gets one new sample per frame. 32 frames → 32 time points.

Usage:
    conda run -n base python3 sim/plot_raw.py [--save] [--seed N]
"""

import os, sys, csv, argparse, random
import numpy as np

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("ERROR: matplotlib not found")

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_CSV    = os.path.join(SCRIPT_DIR, 'sim_decoded.csv')
OUT_PNG    = os.path.join(SCRIPT_DIR, 'raw_data.png')

N_ADC    = 4
N_GROUPS = 16
ADC_MIN  = 512
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


def get_waveform(rows, d, a, g):
    """Return amplitude array across all frames for waveform (d, a, g)."""
    col = f'D{d}_adc{a}_amp{g}'
    vals = []
    for row in rows:
        v = row.get(col, '').strip()
        if v:
            iv = int(v)
            vals.append(iv if ADC_MIN <= iv <= ADC_MAX else np.nan)
        else:
            vals.append(np.nan)
    return np.array(vals)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--save', action='store_true')
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()

    if not os.path.exists(SIM_CSV):
        sys.exit(f"ERROR: {SIM_CSV} not found")

    rows = load_csv(SIM_CSV)
    n_frames = len(rows)

    # Build full list of 512 waveform IDs and pick 8 at random
    all_waveforms = [(d, a, g)
                     for d in range(1, 9)
                     for a in range(1, N_ADC + 1)
                     for g in range(1, N_GROUPS + 1)]

    random.seed(args.seed)
    chosen = random.sample(all_waveforms, 8)
    print(f"Seed {args.seed} — selected waveforms:")
    for d, a, g in chosen:
        print(f"  D{d}_adc{a}_amp{g}")

    x = np.arange(n_frames)

    fig, axes = plt.subplots(8, 1, figsize=(14, 16), sharex=True)
    fig.suptitle(
        f'8 randomly selected waveforms — sim_decoded.csv  ({n_frames} frames)\n'
        f'Each waveform = one independent ADC reading per frame  |  seed={args.seed}',
        fontsize=11
    )

    colors = plt.cm.tab10(np.linspace(0, 0.8, 8))

    for idx, (d, a, g) in enumerate(chosen):
        ax  = axes[idx]
        wav = get_waveform(rows, d, a, g)

        ax.plot(x, wav, '-o', color=colors[idx],
                linewidth=1.3, markersize=4, alpha=0.9)
        ax.set_ylabel(f'D{d} adc{a} g{g}\n(LSB)', fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(True, alpha=0.25)

        # y-axis range with headroom
        valid = wav[~np.isnan(wav)]
        if len(valid):
            mid  = (valid.max() + valid.min()) / 2
            half = max((valid.max() - valid.min()) / 2 * 1.4, 50)
            ax.set_ylim(mid - half, mid + half)

    axes[-1].set_xlabel('Frame index', fontsize=10)
    axes[-1].set_xticks(x[::2])
    plt.tight_layout()

    if args.save:
        plt.savefig(OUT_PNG, dpi=130)
        print(f"Saved → {OUT_PNG}")
    else:
        plt.show()
    plt.close()


if __name__ == '__main__':
    main()
