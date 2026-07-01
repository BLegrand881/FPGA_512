#!/usr/bin/env python3
"""
extract_h5_stim.py  --  Convert a Keysight H5 logic-analyzer capture of the
real ADC serial interface into hex stimulus + golden expected-value files for
the Verilog testbench tb_h5.v.

Signal mapping (confirmed from capture analysis):
  Bit  0 : ADC bit clock  (~21.3 MHz = j33_clk32mhz domain)
  Bit  1 : next_amps_in   (group-boundary pulse, broadcast to all 8 channels)
  Bit  2 : sync_in        (frame sync, quasi-static in this capture)
  Bits 3-10 : data_in[7:0] (8 independent serial ADC channels)
  Bits 11-15: unused (always 0)

Output files (written to same directory as this script, i.e. sim/):
  stim_h5.hex            -- N_CYCLES 10-bit stimulus vectors, one per clock cycle
                              bit[9]   = sync_in  (broadcast to all 8 channels)
                              bit[8]   = next_amps_in (broadcast)
                              bits[7:0]= data_in[7:0]
  expected_ch{0..7}.hex  -- expected 12-bit ADC words per channel, in order
                              Each file has EXP_PER_CH = 512 lines.

Golden values are derived by simulating rx_process_mux's Stage-1 counter and
shift-register logic in Python, using the exact same stim vectors.

Timing note:
  stim[k] is applied by the Verilog testbench at negedge k and sampled by the
  DUT at posedge k+1.  The Python simulation iterates in lockstep: at the
  START of iteration k, cyc_in_group holds the value the DUT registered at
  the previous posedge.  The data-capture and counter update both use stim[k],
  matching the Verilog's non-blocking semantics exactly.

Run from repo root:
  conda run python3 sim/extract_h5_stim.py
"""

import os
import sys

try:
    import h5py
    import numpy as np
except ImportError:
    sys.exit("ERROR: h5py / numpy not found.  Run: conda install h5py numpy")

# ---------------------------------------------------------------------------
# Parameters matching rx_process_mux (top.v instantiation)
# ---------------------------------------------------------------------------
N_FRAMES     = 80         # ADC-frame repetitions to extract
N_GROUPS_FR  = 16         # ADC groups per logical frame
N_GROUPS     = N_FRAMES * N_GROUPS_FR   # 128 total groups
N_CYCLES     = N_GROUPS * 64            # 8192 stimulus clock cycles
ZERO_CYCLES  = 11
N_ADC_PER_GP = 4
ADC_BITS     = 12
GROUP_CYCLES = 64
N_CH         = 8
EXP_PER_CH   = N_GROUPS * N_ADC_PER_GP  # 512 words per channel

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
H5_FILE    = os.path.join(SCRIPT_DIR, '..', 'sine-all-5.h5')

# ---------------------------------------------------------------------------
# Load raw capture
# ---------------------------------------------------------------------------
print(f"Loading {H5_FILE} ...")
with h5py.File(H5_FILE, 'r') as f:
    ds     = f['Waveforms/Digital 0/Digital 0Data']
    n_load = N_CYCLES * 110 + 20_000   # ~110 2GHz-samples per ADC clock cycle
    raw    = ds[:n_load].astype(np.uint32)
print(f"  Loaded {len(raw):,} raw samples")

# ---------------------------------------------------------------------------
# Find ADC clock rising edges, sample signals at each edge
# ---------------------------------------------------------------------------
clk   = (raw & 1).astype(np.int8)
edges = np.where(np.diff(clk) > 0)[0] + 1    # index of each rising edge
print(f"  Found {len(edges):,} clock rising edges")

na_at_edge   = ((raw[edges] >> 1) & 1).astype(np.uint8)    # next_amps
sync_at_edge = ((raw[edges] >> 2) & 1).astype(np.uint8)    # sync
data_at_edge = ((raw[edges] >> 3) & 0xFF).astype(np.uint8) # data_in[7:0]

# ---------------------------------------------------------------------------
# Align to first next_amps rising edge (start of a complete group)
# ---------------------------------------------------------------------------
nxt_rises  = np.where(np.diff(na_at_edge.astype(np.int8)) > 0)[0] + 1
start_edge = int(nxt_rises[0])
end_edge   = start_edge + N_CYCLES

if end_edge > len(edges):
    sys.exit(f"ERROR: need {N_CYCLES} cycles from edge {start_edge}, "
             f"only {len(edges) - start_edge} available. Load more samples.")

print(f"  Aligning at first next_amps rising edge: clock-edge index {start_edge}")
print(f"  Extracting {N_CYCLES} cycles (edges [{start_edge}, {end_edge}))")

stim_na   = na_at_edge  [start_edge:end_edge]
stim_sync = sync_at_edge[start_edge:end_edge]
stim_data = data_at_edge[start_edge:end_edge]

# ---------------------------------------------------------------------------
# Write stim_h5.hex
# ---------------------------------------------------------------------------
stim_path = os.path.join(SCRIPT_DIR, 'stim_h5.hex')
with open(stim_path, 'w') as fh:
    for k in range(N_CYCLES):
        val = (int(stim_sync[k]) << 9) | (int(stim_na[k]) << 8) | int(stim_data[k])
        fh.write(f'{val:03X}\n')
print(f"  Wrote {N_CYCLES} stimulus vectors -> {stim_path}")

# ---------------------------------------------------------------------------
# Simulate DUT Stage-1 counter + shift-register to produce golden outputs
#
# Mirrors rx_process_mux exactly:
#   * cyc_in_group is checked (for in_data_phase) BEFORE being updated --
#     this matches Verilog's non-blocking semantics where all RHS reads use
#     pre-posedge values and all LHS writes take effect afterwards.
#   * SR shift direction: {new_bit, sr[11:1]} -- new bit enters bit[11].
#     After 12 shifts of LSB-first data: sr[11]=MSB, sr[0]=LSB.
# ---------------------------------------------------------------------------
cyc_in_group = [0] * N_CH
sr           = [[0] * N_ADC_PER_GP for _ in range(N_CH)]
expected     = [[] for _ in range(N_CH)]

for k in range(N_CYCLES):
    na   = int(stim_na[k])
    data = int(stim_data[k])

    for ch in range(N_CH):
        cyc       = cyc_in_group[ch]
        in_data   = ZERO_CYCLES <= cyc < ZERO_CYCLES + N_ADC_PER_GP * ADC_BITS

        if in_data:
            bit_idx   = cyc - ZERO_CYCLES
            which_adc = bit_idx % N_ADC_PER_GP
            which_bit = bit_idx // N_ADC_PER_GP
            bit_val   = (data >> ch) & 1

            sr[ch][which_adc] = ((bit_val << 11) | (sr[ch][which_adc] >> 1)) & 0xFFF

            if which_bit == ADC_BITS - 1:
                expected[ch].append(sr[ch][which_adc])

        # Update counter after data capture (non-blocking semantics)
        if na:
            cyc_in_group[ch] = 0
        elif cyc_in_group[ch] < GROUP_CYCLES - 1:
            cyc_in_group[ch] += 1

# Sanity check
ok = True
for ch in range(N_CH):
    n = len(expected[ch])
    status = "OK" if n == EXP_PER_CH else f"WARNING expected {EXP_PER_CH}"
    print(f"  ch{ch}: {n} decoded words  [{status}]")
    if n != EXP_PER_CH:
        ok = False

if not ok:
    print("WARNING: some channels have unexpected word counts -- check alignment")

# ---------------------------------------------------------------------------
# Write expected_ch{0..7}.hex
# ---------------------------------------------------------------------------
for ch in range(N_CH):
    path = os.path.join(SCRIPT_DIR, f'expected_ch{ch}.hex')
    with open(path, 'w') as fh:
        for w in expected[ch]:
            fh.write(f'{w:03X}\n')
    print(f"  Wrote {len(expected[ch])} words -> {path}")

print("Done.")
