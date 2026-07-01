#!/usr/bin/env python3
"""
plot_signal_compare.py  —  Sim vs reference, correct 512-waveform structure.

Structure:
  512 waveforms = 8 D-channels × 4 ADCs × 16 groups (range bins)
  Each waveform gets ONE new sample per frame.

Display: for each D-channel, show all 16 group waveforms (amplitude vs frame)
  with sim (coloured solid lines) overlaid against the best-matching reference
  window (grey dashed lines).  Alignment found by minimising L2 distance on D1.

Usage:
    conda run -n base python3 sim/plot_signal_compare.py [--save]
"""

import os, sys, csv, argparse
import numpy as np

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator
    import matplotlib.cm as cm
except ImportError:
    sys.exit("ERROR: matplotlib not found")

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_CSV    = os.path.join(SCRIPT_DIR, 'sim_decoded.csv')
REF_CSV    = os.path.normpath(os.path.join(SCRIPT_DIR, '..', 'sine-all-5_decoded.csv'))
OUT_PNG    = os.path.join(SCRIPT_DIR, 'signal_compare.png')

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


def group_avg(row, d, g):
    """Mean of 4 ADC values for (D-channel d, group g+1). NaN if all corrupt."""
    vals = []
    for a in range(1, N_ADC + 1):
        col = f'D{d}_adc{a}_amp{g+1}'
        v = row.get(col, '').strip()
        if v:
            iv = int(v)
            if ADC_MIN <= iv <= ADC_MAX:
                vals.append(iv)
    return np.mean(vals) if vals else np.nan


def build_matrix(rows, d):
    """Return array shape (N_GROUPS, n_frames): amplitude per group per frame."""
    n = len(rows)
    mat = np.full((N_GROUPS, n), np.nan)
    for fi, row in enumerate(rows):
        for g in range(N_GROUPS):
            mat[g, fi] = group_avg(row, d, g)
    return mat


def find_ref_offset(sim_rows, ref_rows):
    """Find the reference frame index that best matches sim frame 0, using D1."""
    sim_vec = np.array([group_avg(sim_rows[0], 1, g) for g in range(N_GROUPS)])
    n_sim   = len(sim_rows)
    best_i, best_err = 0, np.inf

    for ri in range(len(ref_rows) - n_sim + 1):
        ref_vec = np.array([group_avg(ref_rows[ri], 1, g) for g in range(N_GROUPS)])
        err = np.nansum((sim_vec - ref_vec) ** 2)
        if err < best_err:
            best_err = err
            best_i   = ri

    return best_i


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--save', action='store_true')
    args = parser.parse_args()

    for p in (SIM_CSV, REF_CSV):
        if not os.path.exists(p):
            sys.exit(f"ERROR: {p} not found")

    sim_rows = load_csv(SIM_CSV)
    ref_rows = load_csv(REF_CSV)
    n_sim = len(sim_rows)
    print(f"Sim: {n_sim} frames    Reference: {len(ref_rows)} frames")

    ref_offset = find_ref_offset(sim_rows, ref_rows)
    print(f"Best reference alignment: ref frames [{ref_offset}..{ref_offset+n_sim-1}]")

    ref_window = ref_rows[ref_offset : ref_offset + n_sim]

    x = np.arange(n_sim)   # frame index 0..11, shared x-axis for direct comparison

    sim_times = np.array([float(r['frame_time']) for r in sim_rows]) * 1e6
    ref_times = np.array([float(r['frame_time']) for r in ref_window]) * 1e6
    sim_period_us = float(np.median(np.diff(sim_times)))
    ref_period_us = float(np.median(np.diff(ref_times)))
    print(f"Sim frame period: {sim_period_us:.1f} µs  |  Ref frame period: {ref_period_us:.1f} µs")

    group_colors = cm.get_cmap('plasma')(np.linspace(0.05, 0.95, N_GROUPS))

    fig, axes = plt.subplots(8, 1, figsize=(16, 18), sharex=True)
    fig.suptitle(
        f'Sim (solid •) vs Reference (dashed ▪) — 12 frames each, frame-aligned\n'
        f'Ref window: frames {ref_offset}–{ref_offset+n_sim-1} of {len(ref_rows)} | '
        f'sim Δt={sim_period_us:.0f} µs/frame  ref Δt={ref_period_us:.0f} µs/frame | '
        f'colour = group (range bin 1–16)',
        fontsize=9
    )

    for idx, d in enumerate(range(1, 9)):
        ax = axes[idx]
        sim_mat = build_matrix(sim_rows,   d)
        ref_mat = build_matrix(ref_window, d)

        for g in range(N_GROUPS):
            c = group_colors[g]
            label = f'g{g+1}' if d == 1 else '_'
            ax.plot(x, sim_mat[g], '-o', color=c, linewidth=1.2,
                    markersize=3.5, alpha=0.9, label=label)
            ax.plot(x, ref_mat[g], '--s', color=c, linewidth=0.8,
                    markersize=2.5, alpha=0.55)

        ax.set_ylabel(f'D{d} (LSB)', fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(True, alpha=0.2)
        ax.yaxis.set_major_locator(MaxNLocator(4))
        ax.set_xlim(-0.3, n_sim - 0.7)

    axes[0].legend(ncol=8, fontsize=6, loc='upper right',
                   title='group (solid=sim, dashed=ref)', title_fontsize=6)
    axes[-1].set_xlabel('Frame index (0 = first frame of each dataset)', fontsize=9)
    axes[-1].set_xticks(x)
    plt.tight_layout()

    if args.save:
        plt.savefig(OUT_PNG, dpi=130)
        print(f"Saved → {OUT_PNG}")
    else:
        plt.show()
    plt.close()


if __name__ == '__main__':
    main()
