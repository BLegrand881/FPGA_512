#!/usr/bin/env python3
"""
debug_deep.py — Deep stim-level analysis of sinusoid outliers.

For each flagged waveform sample:
  1. Examine the full 64-cycle group in stim_h5.hex
  2. Check zero-phase (cycles 0-12): are they actually zero?
  3. Measure na pulse width and exact timing
  4. Extract the actual decoded data bits for that channel
  5. Compute distance from the nearest na edge to the outlier's data bits
  6. Interval analysis: do bad frames repeat at a fixed period?
"""

import os, sys, csv, argparse
import numpy as np

try:
    from scipy.optimize import curve_fit
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_CSV   = os.path.join(SCRIPT_DIR, 'sim_decoded.csv')
STIM_HEX  = os.path.join(SCRIPT_DIR, 'stim_h5.hex')

N_D, N_ADC, N_GROUPS = 8, 4, 16
GROUP_CYCLES  = 64
N_GROUPS_FR   = 16
NA_CYCLES     = 2
ZERO_CYCLES   = 11
DATA_OFFSET   = NA_CYCLES + ZERO_CYCLES   # = 13
ADC_BITS      = 12
ADC_MIN, ADC_MAX = 512, 3584

# lane_framer collects 128 words (mux bursts faster than one ADC frame), then
# EMITS for 132 words × 12 bits = 1584 clk.
# Empirically measured from CSV timing: Δt = 80000 ns = 2560 cycles @ 32 MHz.
# COLLECT_CYCLES = 2560 - 1584 = 976 (not 1024 — mux burst timing).
EMIT_CYCLES    = 132 * 12          # = 1584
FRAME_PERIOD   = 2560              # measured (not calculated 2608)
COLLECT_CYCLES = FRAME_PERIOD - EMIT_CYCLES   # = 976

# frame_time[0] from CSV = 32750 ns = 1048 cycles.
# Empirically: na appears at relative cycle 4 when STIM_OFFSET=60, confirming
# that the actual group start is 4 cycles later → STIM_OFFSET = 64.
# (The 4-cycle shift is the pipeline latency through cb_clk32mhz synchronisation.)
STIM_OFFSET    = 64

SIGMA_THRESH = 3.0
MIN_ABS      = 100.0


# ─── loaders ──────────────────────────────────────────────────────────────────

def load_csv(path):
    rows = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            try: float(row['frame_time'])
            except: continue
            rows.append(row)
    return rows

def load_stim(path):
    vals = []
    with open(path) as f:
        for line in f:
            s = line.strip()
            if s: vals.append(int(s, 16))
    return np.array(vals, dtype=np.int32)

def get_waveform(rows, d, a, g):
    col = f'D{d}_adc{a}_amp{g}'
    out = []
    for row in rows:
        v = row.get(col, '').strip()
        out.append(float(v) if v and ADC_MIN <= int(v) <= ADC_MAX else np.nan)
    return np.array(out)


# ─── sine fit ─────────────────────────────────────────────────────────────────

def fit_sine(x, y):
    valid = ~np.isnan(y)
    if valid.sum() < 6: return None, None
    xv, yv = x[valid], y[valid]
    C0 = np.mean(yv); yc = yv - C0
    fft = np.fft.rfft(yc, n=len(yc)); mag = np.abs(fft); mag[0] = 0
    f0 = float(np.fft.rfftfreq(len(yc))[np.argmax(mag)]) if mag.max() > 0 else 0.1
    f0 = max(1e-4, min(f0, 0.49))
    A0 = float(np.std(yc) * np.sqrt(2))
    def model(xv, A, f, phi, C): return A * np.sin(2*np.pi*f*xv+phi) + C
    if HAS_SCIPY:
        try:
            popt, _ = curve_fit(model, xv, yv,
                                p0=[A0, f0, 0, C0],
                                bounds=([-4096,.0001,-np.pi,0],[4096,.5,np.pi,4096]),
                                maxfev=5000)
        except: popt = [A0, f0, 0, C0]
    else:
        popt = [A0, f0, 0, C0]
    yfit = np.full_like(y, np.nan)
    yfit[valid] = model(xv, *popt)
    return yfit, np.where(valid, y - yfit, np.nan)


# ─── stim context helpers ─────────────────────────────────────────────────────

def group_stim_slice(stim, fi, g_idx, pad_before=5, pad_after=5):
    """Return (start_k, array of stim values) for group g_idx in frame fi,
       plus pad_before/pad_after extra cycles.
       group_start = STIM_OFFSET + fi * FRAME_PERIOD + g_idx * GROUP_CYCLES
       STIM_OFFSET = 60: reset + pipeline delay before first collect begins.
       FRAME_PERIOD = 2560: empirically measured (not 2608)."""
    gs = STIM_OFFSET + fi * FRAME_PERIOD + g_idx * GROUP_CYCLES
    k0 = max(0, gs - pad_before)
    k1 = min(len(stim), gs + GROUP_CYCLES + pad_after)
    return gs, stim[k0:k1], k0

def decode_stim(val):
    return {
        'data': int(val) & 0xFF,
        'na':   (int(val) >> 8) & 1,
        'sync': (int(val) >> 9) & 1,
    }


def analyze_group(stim, fi, g_idx, a_idx, d_idx):
    """
    Deep analysis of one (frame, group, adc, d_channel) outlier.
    Returns a dict of findings.
    """
    gs, window, w_start = group_stim_slice(stim, fi, g_idx, pad_before=10, pad_after=10)

    # Build cycle-by-cycle view of the 64-cycle group
    cyc_data  = []
    for cyc in range(GROUP_CYCLES):
        k = gs + cyc
        if 0 <= k < len(stim):
            cyc_data.append(decode_stim(stim[k]))
        else:
            cyc_data.append({'data': 0, 'na': 0, 'sync': 0})

    # 1. na pulse characterisation inside this group
    na_seq  = [c['na'] for c in cyc_data]
    na_ones = [i for i,v in enumerate(na_seq) if v]
    na_width     = len(na_ones)
    na_first     = na_ones[0] if na_ones else None
    na_last      = na_ones[-1] if na_ones else None

    # 2. zero phase check (cycles 0..DATA_OFFSET-1 should all have data=0)
    zero_phase   = cyc_data[:DATA_OFFSET]
    nonzero_in_zero = [(i, c['data']) for i,c in enumerate(zero_phase) if c['data'] != 0]

    # 3. data bits for our (adc, d_channel)
    # The decoder has a 1-group pipeline latency: the decoded CSV value for
    # amplitude group g corresponds to stim group g-1.  So we read g_idx-1.
    data_bits = []
    for bit_k in range(ADC_BITS):
        cyc     = DATA_OFFSET + a_idx + bit_k * N_ADC
        if cyc < GROUP_CYCLES:
            c   = cyc_data[cyc]
            bit = (c['data'] >> d_idx) & 1
            data_bits.append((cyc, bit, c['na'], c['sync'], c['data']))

    # Reconstruct what the decoded word would be (LSB-first, new bit → bit[11])
    sr = 0
    for _, bit, _, _, _ in data_bits:
        sr = ((bit << 11) | (sr >> 1)) & 0xFFF
    reconstructed = sr

    # 4. Distance from last na=1 to first data bit
    first_data_cyc = DATA_OFFSET + a_idx  # = 13+a_idx
    dist_na_to_data = first_data_cyc - (na_last if na_last is not None else 0)

    # 5. Trailing cycles (after data, before end of group)
    trail_start = DATA_OFFSET + N_ADC * ADC_BITS  # = 13+48 = 61
    trail_nonzero = [(i, cyc_data[i]['data'])
                     for i in range(trail_start, GROUP_CYCLES)
                     if cyc_data[i]['data'] != 0]

    # 6. Look-ahead: does na from next group intrude?
    next_gs = gs + GROUP_CYCLES
    next_na_early = []
    for k in range(next_gs, min(next_gs + 5, len(stim))):
        v = decode_stim(stim[k])
        if v['na']: next_na_early.append(k - gs)

    return {
        'na_width':         na_width,
        'na_first':         na_first,
        'na_last':          na_last,
        'na_ones':          na_ones,
        'nonzero_in_zero':  nonzero_in_zero,
        'data_bits':        data_bits,
        'reconstructed':    reconstructed,
        'dist_na_to_data':  dist_na_to_data,
        'trail_nonzero':    trail_nonzero,
        'next_na_early':    next_na_early,
    }


# ─── main ─────────────────────────────────────────────────────────────────────

def main():
    for p in (SIM_CSV, STIM_HEX):
        if not os.path.exists(p):
            sys.exit(f'ERROR: {p} not found')

    rows  = load_csv(SIM_CSV)
    stim  = load_stim(STIM_HEX)
    n_fr  = len(rows)
    print(f'Frames: {n_fr}   Stim cycles: {len(stim)}')

    x = np.arange(n_fr, dtype=float)

    # ── collect outliers ──────────────────────────────────────────────────────
    outliers = []
    for d in range(1, N_D+1):
        for a in range(1, N_ADC+1):
            for g in range(1, N_GROUPS+1):
                y    = get_waveform(rows, d, a, g)
                yfit, res = fit_sine(x, y)
                if yfit is None: continue
                vr   = res[~np.isnan(res)]
                sig  = float(np.std(vr)) if len(vr) >= 2 else 0
                thr  = max(SIGMA_THRESH * sig, MIN_ABS)
                for fi in range(n_fr):
                    if np.isnan(res[fi]): continue
                    if abs(res[fi]) > thr:
                        outliers.append(dict(d=d, a=a, g=g, fi=fi,
                                             value=float(y[fi]),
                                             fitted=float(yfit[fi]),
                                             residual=float(res[fi]),
                                             sigma=sig))

    print(f'Outliers: {len(outliers)}\n')

    # ── deep stim analysis ────────────────────────────────────────────────────
    records = []
    zero_phase_dirty   = 0
    trail_dirty        = 0
    na_width_wrong     = 0   # != 2
    na_not_at_0        = 0   # na doesn't start at cycle 0
    reconstruct_match  = 0
    reconstruct_mismatch = 0
    dist_na_hist       = {}

    for o in outliers:
        d,a,g,fi = o['d'],o['a'],o['g'],o['fi']
        # g-2: g is 1-indexed, -1 to 0-index, -1 more for 1-group decoder latency
        info = analyze_group(stim, fi, g-2, a-1, d-1)

        # zero-phase dirty?
        if info['nonzero_in_zero']:
            zero_phase_dirty += 1
        if info['trail_nonzero']:
            trail_dirty += 1
        if info['na_width'] != 2:
            na_width_wrong += 1
        if info['na_first'] not in (0, 1, None):
            na_not_at_0 += 1

        dist = info['dist_na_to_data']
        dist_na_hist[dist] = dist_na_hist.get(dist, 0) + 1

        # does the reconstructed word from stim bits match the decoded CSV value?
        decoded_val = int(o['value']) if not np.isnan(o['value']) else -1
        match = (info['reconstructed'] == decoded_val)
        if match: reconstruct_match += 1
        else:     reconstruct_mismatch += 1

        records.append({**o, **info, 'stim_match': match, 'decoded_val': decoded_val})

    N = len(outliers)

    # ── frame interval analysis ───────────────────────────────────────────────
    bad_frames = sorted(set(o['fi'] for o in outliers))
    gaps = np.diff(bad_frames) if len(bad_frames) > 1 else np.array([])

    # ── value clustering ─────────────────────────────────────────────────────
    vals = np.array([o['value'] for o in outliers if not np.isnan(o['value'])])
    # Bucket: near-floor (<1200), mid (1200-1600), near-fitted (>1600)
    near_floor = int((vals < 1200).sum())
    mid_range  = int(((vals >= 1200) & (vals < 1600)).sum())
    near_fit   = int((vals >= 1600).sum())

    # ── zero-phase bit analysis per D-channel ────────────────────────────────
    # Which D-channels have non-zero bits during zero phase?
    zero_phase_by_d = {d: 0 for d in range(1,9)}
    for rec in records:
        for cyc, data in rec['nonzero_in_zero']:
            # Check each D-channel bit
            for bit in range(8):
                if (data >> bit) & 1:
                    zero_phase_by_d[bit+1] += 1

    # ── na timing per outlier ────────────────────────────────────────────────
    na_timing_rows = []
    for rec in records[:50]:   # detailed look at worst 50
        na_timing_rows.append((
            rec['fi'], rec['d'], rec['a'], rec['g'],
            rec['na_first'], rec['na_last'], rec['na_width'],
            rec['dist_na_to_data'],
            len(rec['nonzero_in_zero']),
            rec['stim_match'],
            int(rec['value']) if not np.isnan(rec['value']) else -1,
        ))

    # Sort by residual magnitude
    records_sorted = sorted(records, key=lambda r: abs(r['residual']), reverse=True)

    # ─── PRINT REPORT ─────────────────────────────────────────────────────────
    print('='*72)
    print(f'  DEEP STIM ANALYSIS  ({N} outliers)')
    print('='*72)

    print(f'\n── Value distribution ──────────────────────────────────────────────')
    print(f'  Near floor (<1200 LSB) : {near_floor:4d}  ({100*near_floor/N:.0f}%)  '
          f'← whole word pulled near ADC_MIN')
    print(f'  Mid-range  (1200-1600) : {mid_range:4d}  ({100*mid_range/N:.0f}%)')
    print(f'  Near fitted (>1600)    : {near_fit:4d}  ({100*near_fit/N:.0f}%)  '
          f'← small deviations')

    print(f'\n── na pulse characterisation ───────────────────────────────────────')
    print(f'  na_width != 2 (unexpected pulse width) : {na_width_wrong:4d}  ({100*na_width_wrong/N:.0f}%)')
    print(f'  na not starting at cycle 0             : {na_not_at_0:4d}  ({100*na_not_at_0/N:.0f}%)')
    print(f'  Distance na_last→first_data_bit histogram:')
    for dist, cnt in sorted(dist_na_hist.items()):
        print(f'    dist={dist:3d}  count={cnt:4d}  {"█"*min(cnt,40)}')

    print(f'\n── Zero-phase integrity (cycles 0-12 should be data=0x00) ──────────')
    print(f'  Groups with non-zero data in zero phase : {zero_phase_dirty:4d} / {N}  ({100*zero_phase_dirty/N:.0f}%)')
    print(f'  Non-zero bits by D-channel during zero phase:')
    for d, cnt in zero_phase_by_d.items():
        bar = '█'*min(cnt,30) if cnt else ''
        print(f'    D{d}: {cnt:4d}  {bar}')

    print(f'\n── Trailing zeros integrity (cycles 61-63 should be data=0x00) ─────')
    print(f'  Groups with non-zero trailing data : {trail_dirty:4d} / {N}  ({100*trail_dirty/N:.0f}%)')

    print(f'\n── Stim bit reconstruction vs decoded CSV value ─────────────────────')
    print(f'  Stim bits reconstruct to same decoded value : {reconstruct_match:4d} / {N}')
    print(f'  Mismatch (stim and CSV disagree)            : {reconstruct_mismatch:4d} / {N}')
    print(f'  → If MATCH: the decoder is correct; the stim input is the bad data')
    print(f'  → If MISMATCH: the decoder itself introduced the error')

    print(f'\n── Frame interval analysis ──────────────────────────────────────────')
    print(f'  Bad frame indices : {bad_frames}')
    if len(gaps):
        print(f'  Frame gaps        : {gaps.tolist()}')
        print(f'  Gap stats: min={gaps.min()}  max={gaps.max()}  '
              f'median={np.median(gaps):.0f}  mode={int(np.bincount(gaps).argmax())}')
        # Check for periodic pattern
        for period in [2, 3, 4, 5, 6, 8, 10, 16]:
            hits = int((gaps % period == 0).sum())
            if hits / len(gaps) > 0.5:
                print(f'  *** {hits}/{len(gaps)} gaps are multiples of {period} → likely period={period} frames')

    print(f'\n── Top 20 worst outliers — detailed ─────────────────────────────────')
    print(f'  {"fi":>3} D{"a":>2}{"g":>3}  val  fit  resid  '
          f'na_w na@  dist  0ph  trail  match  reconstructed')
    print('  ' + '-'*78)
    for r in records_sorted[:20]:
        print(f'  {r["fi"]:>3} D{r["d"]} a{r["a"]} g{r["g"]:>2}'
              f'  {int(r["value"]) if not np.isnan(r["value"]) else "NaN":>4}'
              f'  {int(r["fitted"]):>4}'
              f'  {r["residual"]:>+6.0f}'
              f'  na_w={r["na_width"]}'
              f'  na@{r["na_first"] if r["na_first"] is not None else "?":>2}'
              f'  d={r["dist_na_to_data"]:>2}'
              f'  0ph={len(r["nonzero_in_zero"])}'
              f'  tr={len(r["trail_nonzero"])}'
              f'  {"✓" if r["stim_match"] else "✗"}'
              f'  recon=0x{r["reconstructed"]:03X}({r["reconstructed"]})')

    print(f'\n── Zero-phase sample (worst outlier, cycle-by-cycle) ────────────────')
    r = records_sorted[0]
    gs, window, w0 = group_stim_slice(stim, r['fi'], r['g']-2, 5, 5)  # g-2: 1-indexed + 1-group latency
    print(f'  Outlier: fi={r["fi"]} D{r["d"]} a{r["a"]} g{r["g"]}  '
          f'val={int(r["value"]) if not np.isnan(r["value"]) else "NaN"}  '
          f'fitted={int(r["fitted"])}')
    print(f'  {"cyc":>4}  {"na":>2}  {"sync":>4}  {"data(hex)":>9}  phase')
    for cyc in range(GROUP_CYCLES):
        k = gs + cyc
        if k >= len(stim): break
        s = decode_stim(stim[k])
        if cyc < DATA_OFFSET:
            phase = f'ZERO[{cyc}]'
        elif cyc < DATA_OFFSET + N_ADC * ADC_BITS:
            bi = cyc - DATA_OFFSET
            phase = f'DATA bit_idx={bi} adc={(bi%N_ADC)+1} bit={(bi//N_ADC)}'
        else:
            phase = f'TRAIL[{cyc}]'
        flag = ' ← NON-ZERO ZERO' if cyc < DATA_OFFSET and s['data'] != 0 else ''
        flag = flag or (' ← na=1 in data!' if s['na'] and DATA_OFFSET <= cyc < DATA_OFFSET+N_ADC*ADC_BITS else '')
        show = (s['na'] or s['sync'] or cyc < DATA_OFFSET or
                cyc >= DATA_OFFSET + N_ADC*ADC_BITS - 2)
        if show:
            print(f'  {cyc:>4}  {s["na"]:>2}  {s["sync"]:>4}  0x{s["data"]:02X}  {phase}{flag}')

    print(f'\n{"="*72}')


if __name__ == '__main__':
    main()
