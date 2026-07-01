#!/usr/bin/env python3
"""
debug_outliers.py  —  Sinusoid-fit outlier detection for 512 ADC waveforms.

For each of the 512 independent waveforms (8 D-channels × 4 ADC chips × 16
groups) in sim_decoded.csv:
  1. Fit a sinusoid (A·sin(2πft + φ) + C) across the decoded frames.
  2. Flag samples whose |residual| exceeds 3σ (minimum 100 LSB).
  3. Map each outlier back to stim_h5.hex: retrieve the actual stim words
     (data, na, sync) for all 12 bits of that ADC word.
  4. Report cluster analysis: do outliers concentrate at specific frames,
     D-channels, ADC chips, groups, or bit positions within the 48-bit window?

Usage:
    conda run -n base python3 sim/debug_outliers.py [--top N] [--threshold T]
"""

import os, sys, csv, argparse
import numpy as np

try:
    from scipy.optimize import curve_fit
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_CSV    = os.path.join(SCRIPT_DIR, 'sim_decoded.csv')
STIM_HEX   = os.path.join(SCRIPT_DIR, 'stim_h5.hex')

# Protocol constants (must match extract_h5_stim.py / tb_h5.v)
N_D          = 8
N_ADC        = 4      # ADC chips per group
N_GROUPS     = 16     # range bins per frame
GROUP_CYCLES = 64
N_GROUPS_FR  = 16
NA_CYCLES    = 2      # na=1 pulse width at group start
ZERO_CYCLES  = 11     # zero-data cycles after na goes low
DATA_OFFSET  = NA_CYCLES + ZERO_CYCLES  # = 13; first data bit within group
ADC_BITS     = 12

ADC_MIN = 512
ADC_MAX = 3584


# =============================================================================
# Data loading
# =============================================================================

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


def load_stim(path):
    stim = []
    with open(path) as f:
        for line in f:
            s = line.strip()
            if s:
                stim.append(int(s, 16))
    return np.array(stim, dtype=np.uint16)


def get_waveform(rows, d, a, g):
    """Extract amplitude array (NaN for missing/corrupted) for waveform (d,a,g)."""
    col  = f'D{d}_adc{a}_amp{g}'
    vals = []
    for row in rows:
        v = row.get(col, '').strip()
        if v:
            iv = int(v)
            vals.append(float(iv) if ADC_MIN <= iv <= ADC_MAX else np.nan)
        else:
            vals.append(np.nan)
    return np.array(vals)


# =============================================================================
# Sinusoid fitting
# =============================================================================

def _sine_model(x, A, f, phi, C):
    return A * np.sin(2.0 * np.pi * f * x + phi) + C


def fit_sine(x, y, min_pts=6):
    """
    Fit y ≈ A·sin(2πfx + φ) + C.
    Returns (y_fitted, residuals) on the full x grid (NaN where y is NaN),
    or (None, None) if fitting is not possible.
    """
    valid = ~np.isnan(y)
    n_valid = int(valid.sum())
    if n_valid < min_pts:
        return None, None

    xv = x[valid]
    yv = y[valid]
    C0 = float(np.mean(yv))
    yc = yv - C0
    A0 = float(np.std(yc) * np.sqrt(2))

    # Initial frequency estimate from FFT of valid samples
    fft = np.fft.rfft(yc, n=n_valid)
    freqs = np.fft.rfftfreq(n_valid)
    mag = np.abs(fft)
    mag[0] = 0.0
    f0 = float(freqs[np.argmax(mag)]) if mag.max() > 0 else 0.1
    if f0 <= 0 or f0 >= 0.5:
        f0 = 0.1

    if HAS_SCIPY:
        try:
            popt, _ = curve_fit(
                _sine_model, xv, yv,
                p0=[A0, f0, 0.0, C0],
                bounds=([-4096, 1e-4, -np.pi, 0.0], [4096, 0.5, np.pi, 4096.0]),
                maxfev=5000,
            )
            A_fit, f_fit, phi_fit, C_fit = popt
        except Exception:
            # Fall back to FFT-derived parameters
            A_fit, f_fit, phi_fit, C_fit = A0, f0, 0.0, C0
    else:
        # Without scipy: use FFT amplitude + phase
        k_peak = int(np.argmax(mag))
        A_fit  = 2.0 * float(np.abs(fft[k_peak])) / n_valid
        phi_fit = float(np.angle(fft[k_peak]))
        f_fit  = f0
        C_fit  = C0

    y_fit = np.full_like(y, np.nan)
    y_fit[valid] = _sine_model(xv, A_fit, f_fit, phi_fit, C_fit)
    residuals = np.where(valid, y - y_fit, np.nan)
    return y_fit, residuals


# =============================================================================
# Stim context lookup
# =============================================================================

def get_stim_context(stim, fi, g_idx, a_idx):
    """
    Return list of dicts for all 12 bits of ADC word (frame fi, group g_idx,
    adc a_idx) — all 0-indexed.

    Each dict:  stim_k, bit_pos_in_48, na, sync, data_byte, data_bit
    bit_pos_in_48 = position of this bit in the 48-bit data window.
    """
    group_start = fi * N_GROUPS_FR * GROUP_CYCLES + g_idx * GROUP_CYCLES
    bits_info = []
    for bit_k in range(ADC_BITS):
        # bit_idx within data phase = a_idx + bit_k * N_ADC
        bit_idx  = a_idx + bit_k * N_ADC          # 0..47
        stim_k   = group_start + DATA_OFFSET + bit_idx
        if stim_k >= len(stim):
            continue
        val      = int(stim[stim_k])
        na       = (val >> 8) & 1
        sync     = (val >> 9) & 1
        data_byte = val & 0xFF
        data_bit  = (data_byte >> (a_idx // N_ADC)) & 1  # this ADC's actual bit
        # Note: D-channel d maps to bit (d-1) of data_byte; here we just report
        # the full byte and the bit position.
        bits_info.append(dict(
            stim_k=stim_k,
            bit_pos=bit_idx,
            bit_k=bit_k,
            na=na,
            sync=sync,
            data_byte=data_byte,
        ))
    return bits_info


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description='Sinusoid-fit outlier analysis')
    parser.add_argument('--top',       type=int,   default=30,
                        help='Number of worst outliers to list (default: 30)')
    parser.add_argument('--threshold', type=float, default=3.0,
                        help='Outlier threshold in sigma (default: 3.0)')
    parser.add_argument('--min-abs',   type=float, default=100.0,
                        help='Minimum absolute residual to count as outlier (default: 100 LSB)')
    args = parser.parse_args()

    # ── Load ─────────────────────────────────────────────────────────────────
    for p in (SIM_CSV, STIM_HEX):
        if not os.path.exists(p):
            sys.exit(f'ERROR: {p} not found')

    rows = load_csv(SIM_CSV)
    stim = load_stim(STIM_HEX)
    n_frames = len(rows)
    print(f'sim_decoded.csv : {n_frames} frames')
    print(f'stim_h5.hex     : {len(stim)} cycles  '
          f'({len(stim)//(N_GROUPS_FR*GROUP_CYCLES)} ADC frames)')
    if not HAS_SCIPY:
        print('WARNING: scipy not found — using FFT-only fit (less accurate)')

    x = np.arange(n_frames, dtype=float)

    # ── Fit and collect outliers ──────────────────────────────────────────────
    print('\nFitting sinusoids to 512 waveforms ...')
    outliers    = []   # list of dicts
    n_fit_ok    = 0
    n_skipped   = 0

    for d in range(1, N_D + 1):
        for a in range(1, N_ADC + 1):
            for g in range(1, N_GROUPS + 1):
                y = get_waveform(rows, d, a, g)
                yfit, residuals = fit_sine(x, y)
                if yfit is None:
                    n_skipped += 1
                    continue
                n_fit_ok += 1

                valid_res = residuals[~np.isnan(residuals)]
                sigma = float(np.std(valid_res)) if len(valid_res) >= 2 else 0.0
                thresh = max(args.threshold * sigma, args.min_abs)

                for fi in range(n_frames):
                    if np.isnan(residuals[fi]):
                        continue
                    r = float(residuals[fi])
                    if abs(r) > thresh:
                        outliers.append(dict(
                            d=d, a=a, g=g, fi=fi,
                            value=float(y[fi]),
                            fitted=float(yfit[fi]),
                            residual=r,
                            sigma=sigma,
                        ))

    print(f'  Fitted  : {n_fit_ok} waveforms')
    print(f'  Skipped : {n_skipped} waveforms (too few valid frames)')
    print(f'  Outliers: {len(outliers)} samples flagged '
          f'(|residual| > {args.threshold}σ, min {args.min_abs:.0f} LSB)')

    if not outliers:
        print('\nNo outliers — decoder output looks sinusoidally consistent.')
        return

    # ── Stim cross-reference ──────────────────────────────────────────────────
    print('\nCross-referencing with stim_h5.hex ...')

    bit_pos_counts  = np.zeros(N_ADC * ADC_BITS, dtype=int)  # 48 positions
    group_counts    = np.zeros(N_GROUPS, dtype=int)
    d_counts        = np.zeros(N_D, dtype=int)
    a_counts        = np.zeros(N_ADC, dtype=int)
    frame_counts    = np.zeros(n_frames, dtype=int)
    na_in_window    = 0    # stim shows na=1 during this word's bit cycles
    sync_in_window  = 0

    enriched = []

    for o in outliers:
        d_i, a_i, g_i, fi = o['d']-1, o['a']-1, o['g']-1, o['fi']
        ctx = get_stim_context(stim, fi, g_i, a_i)

        any_na   = any(c['na']   for c in ctx)
        any_sync = any(c['sync'] for c in ctx)
        bp_list  = [c['bit_pos'] for c in ctx]

        for bp in bp_list:
            bit_pos_counts[bp] += 1
        group_counts[g_i]  += 1
        d_counts[d_i]      += 1
        a_counts[a_i]      += 1
        frame_counts[fi]   += 1
        if any_na:
            na_in_window  += 1
        if any_sync:
            sync_in_window += 1

        enriched.append({**o,
            'any_na':   any_na,
            'any_sync': any_sync,
            'bit_range': f'{min(bp_list)}-{max(bp_list)}' if bp_list else '?',
        })

    # ── Print report ──────────────────────────────────────────────────────────
    N = len(outliers)
    print(f'\n{"="*70}')
    print(f'  OUTLIER ANALYSIS  —  {N} outlier samples  '
          f'across {n_fit_ok} waveforms')
    print(f'{"="*70}')

    print(f'\n  Stim context flags:')
    print(f'    na=1 during data bits  : {na_in_window:4d}  ({100*na_in_window/N:.0f}%)')
    print(f'    sync=1 during data bits: {sync_in_window:4d}  ({100*sync_in_window/N:.0f}%)')

    print(f'\n  By frame index  (top 10 worst):')
    top_frames = np.argsort(frame_counts)[::-1][:10]
    for fi in top_frames:
        if frame_counts[fi] > 0:
            print(f'    frame {fi:2d} : {frame_counts[fi]:4d} outliers')

    print(f'\n  By D-channel:')
    for i in range(N_D):
        bar = '#' * min(frame_counts.max() or 1, 40) if False else ''
        c = d_counts[i]
        print(f'    D{i+1} : {c:4d}  {"█"*min(c,40)}')

    print(f'\n  By ADC chip:')
    for i in range(N_ADC):
        c = a_counts[i]
        print(f'    adc{i+1} : {c:4d}  {"█"*min(c,40)}')

    print(f'\n  By group (range bin):')
    for i in range(N_GROUPS):
        c = group_counts[i]
        if c > 0:
            print(f'    amp{i+1:2d} : {c:4d}  {"█"*min(c,40)}')

    print(f'\n  Bit-position histogram  (within 48-bit data window):')
    print(f'  {"pos":>3}  {"adc":>4}  {"bit#":>4}  {"count":>6}  bar')
    for bp in range(N_ADC * ADC_BITS):
        c = bit_pos_counts[bp]
        if c > 0:
            adc_chip   = bp % N_ADC
            bit_in_word = bp // N_ADC
            print(f'    {bp:2d}   adc{adc_chip+1}  bit{bit_in_word:2d}  {c:5d}  {"█"*min(c,30)}')

    print(f'\n  Top {min(args.top, N)} worst outliers:')
    sorted_out = sorted(enriched, key=lambda r: abs(r['residual']), reverse=True)
    hdr = (f'  {"fi":>3}  {"D":>2}  {"a":>3}  {"g":>3}'
           f'  {"value":>6}  {"fitted":>7}  {"resid":>8}  {"sigma":>6}'
           f'  na  sync  bits')
    print(hdr)
    print('  ' + '-'*(len(hdr)-2))
    for r in sorted_out[:args.top]:
        print(f'  {r["fi"]:>3}  D{r["d"]}  a{r["a"]}  g{r["g"]:>2}'
              f'  {r["value"]:>6.0f}  {r["fitted"]:>7.0f}  {r["residual"]:>+8.0f}'
              f'  {r["sigma"]:>6.1f}'
              f'  {"Y" if r["any_na"] else "N":>2}  {"Y" if r["any_sync"] else "N":>4}'
              f'  {r["bit_range"]}')

    print(f'\n{"="*70}')

    # ── Hypothesis summary ────────────────────────────────────────────────────
    print('\n  HYPOTHESIS CHECK:')

    # Na-related?
    na_frac = na_in_window / N
    if na_frac > 0.5:
        print(f'  [!] {100*na_frac:.0f}% of outliers have na=1 in their stim window '
              f'-> na-timing issue likely')
    else:
        print(f'  [ ] Only {100*na_frac:.0f}% have na=1 in window -> na timing not '
              f'the primary cause')

    # D-channel bias?
    d_max_frac = d_counts.max() / N
    d_max_idx  = int(d_counts.argmax())
    if d_max_frac > 0.5:
        print(f'  [!] {100*d_max_frac:.0f}% of outliers are on D{d_max_idx+1} '
              f'-> one channel is problematic')
    else:
        print(f'  [ ] Outliers spread across D-channels (max {100*d_max_frac:.0f}% on '
              f'D{d_max_idx+1}) -> not single-channel issue')

    # Frame bias?
    f_max_frac = frame_counts.max() / N
    f_max_idx  = int(frame_counts.argmax())
    if f_max_frac > 0.3:
        print(f'  [!] {100*f_max_frac:.0f}% of outliers at frame {f_max_idx} '
              f'-> specific frames corrupted')
    else:
        print(f'  [ ] Outliers distributed across frames (max {100*f_max_frac:.0f}% '
              f'at frame {f_max_idx})')

    # Bit-position bias?
    if bit_pos_counts.max() > 0:
        bp_max_frac = bit_pos_counts.max() / N
        bp_max_idx  = int(bit_pos_counts.argmax())
        if bp_max_frac > 0.3:
            print(f'  [!] {100*bp_max_frac:.0f}% of outlier bits at position '
                  f'{bp_max_idx} (adc{bp_max_idx%4+1}, bit{bp_max_idx//4}) '
                  f'-> bit-position correlation')
        else:
            print(f'  [ ] Bit positions spread uniformly -> no single-bit bias')

    print()


if __name__ == '__main__':
    main()
