# FT600 Throughput Optimization Notes

## Current Status
- **Data integrity: PASS** — zero errors with settle-after-accept writer
- **Throughput: 0.028 MB/s** — far below theoretical ~35 MB/s (USB 2.0 HS)
- The settle cycle (pause 1 clock after each accepted write) kills throughput

## Root Cause of All Errors
The async FIFO uses non-blocking pointer advance (`rptr_bin <= rptr_bin + 1`).
When `fifo_ren=1` and `data_out <= fifo_rdata` happen on the **same clock edge**,
`fifo_rdata` still shows the **old** word (pre-advance) because non-blocking
assignments evaluate RHS with pre-edge values.

This means: every time we advance the FIFO and sample rdata simultaneously,
we get the word we just consumed, not the next word.

## Approaches Tried

### 1. Direct combinational (first attempt)
```verilog
wire can_send = ~txe_n & ~fifo_empty;
assign fifo_ren = can_send;
data_out <= fifo_rdata;  // stale on advance edges
```
**Result:** ~2000 errors per 10 MB. 1 word skipped at irregular intervals (~1500 words).
Skips happen because fifo_ren advances the pointer, but data_out gets stale rdata.
The FT600 captures stale data, then the next fresh word is 1 ahead → skip.

### 2. Registered TXE_N (`txe_n_r`)
```verilog
reg txe_n_r;
always @(posedge clk) txe_n_r <= txe_n;
if (!txe_n_r) begin wr_n <= 0; ... end
```
**Result:** 2 words skipped per 2048 words (4KB FT600 buffer boundary).
The 1-cycle TXE_N delay means 1 extra write after buffer full + 1 from the
pipeline. Counter increments for both but FT600 doesn't capture.

### 3. Mixed registered + live TXE_N
```verilog
wire can_write = ~txe_n_r;      // drive decision on registered
wire write_ok  = can_write & ~txe_n;  // counter advance on both
```
**Result:** 1 word skipped per 2048 words. Improved from 2 to 1 skip.

### 4. Live TXE_N only (no register)
```verilog
if (!txe_n) begin wr_n <= 0; data_out <= counter; counter <= counter + 1; end
```
**Result:** 1 word skipped per 2048 words. Same as #3 — the skip is from the
registered WR_N pipeline, not the TXE_N sampling.

### 5. Holding register with `accepted` signal
```verilog
wire accepted = ~wr_n & ~txe_n;
assign fifo_ren = accepted;
// Various holding register strategies
```
**Result:** Duplicates instead of skips. The holding register re-drives the
consumed word because fifo_rdata is stale on the advance edge.

### 6. State machine (IDLE → PREFETCH → WRITE)
**Result:** Both skips AND duplicates. More complex, more bugs, worse.

### 7. Two-phase (LATCH → DRIVE with fifo_ren in DRIVE)
```verilog
assign fifo_ren = phase & ~txe_n & ~fifo_empty;  // advance in DRIVE
// LATCH: word <= fifo_rdata
// DRIVE: data_out <= word, fifo_ren fires
```
**Result:** Every other word skipped (0x0001, 0x0003, 0x0005...).
fifo_ren is a reg, so its value persists into the LATCH cycle causing
a spurious advance.

### 8. Settle-after-accept
```verilog
wire accepted = ~wr_n & ~txe_n;
assign fifo_ren = accepted;  // advance only on confirmed capture

if (accepted)
    wr_n <= 1;  // pause 1 cycle for pointer to settle
else if (~fifo_empty & ~txe_n)
    data_out <= fifo_rdata;  // safe: pointer is stable
```
**Result:** ZERO errors in direct-counter mode. But throughput = 0.028 MB/s in
earlier firmware (likely other bugs). Re-tested after USB path fixes: ~22 MB/s
but **same duplicate errors** as approach #9 (~5–7 per 10 MB) when used with
the async FIFO. The settle cycle does NOT fix the underlying CDC issue.

### 9. Confirmed-accept with counter bypass (CURRENT — ft600_streamer.v, WORKING)
```verilog
wire accepted = ~wr_n & ~txe_n;
data_out <= accepted ? (counter + 1) : counter;
if (accepted) counter <= counter + 1;
```
**Result:** ZERO errors, 35 MB/s (USB 2.0). Counter only advances on confirmed
capture. Combinational bypass (`counter + 1`) keeps the pipeline full with no
settle cycle. Eliminates the 1-skip-per-2048 boundary error completely.

### 10. Look-ahead bypass with async FIFO (CURRENT — ft600_writer.v, ~4–7 dupes/10 MB)
```verilog
wire accepted = ~wr_n & ~txe_n;
assign fifo_ren = accepted;
data_out <= accepted ? fifo_rdata_next : fifo_rdata;
// fifo_rdata_next = mem[rptr_bin + 1]  (added to async_fifo.v)
// almost_empty guard prevents reading invalid rdata_next on last word
```
**Result:** ~22 MB/s throughput, but 4–7 duplicate errors per 10 MB.
All errors are `got = expected - 1` (duplicates, not skips). The settle-after-
accept approach (#8) produces the **identical error pattern**, proving this is
NOT a timing/combinational-path issue in the look-ahead MUX. The root cause
is in the async FIFO's CDC read path — see "Open Issue" below.

## Key Lessons

### Verilog / FPGA
1. **Non-blocking assignments** mean `fifo_rdata` is stale on the same edge as `fifo_ren`.
   You CANNOT advance and sample in the same cycle.
2. **Combinational wires vs registered signals**: `fifo_ren` as a wire (`assign`) fires
   immediately; as a `reg` it persists into the next cycle causing spurious advances.
3. **The FTDI reference design** avoids this by using a LOCAL RAM with INDEPENDENT
   read and memory-enable signals. `adv_c_rptr` (pointer advance) and `mem_rden`
   (memory read) are separate. The pointer only advances when TXE_N is confirmed low.
   The memory always shows data at the current pointer. No stale data issue because
   the RAM output is combinational from a pointer that only advances on confirmed writes.
4. **IO_TYPE must match VCCIO**: Bank 2 = 1.5V → LVCMOS15, Banks 6/7 = 2.5V → LVCMOS25.
   Wrong IO_TYPE causes input threshold failures (signals unreadable).
5. **FT600 requires `FT_SetStreamPipe`** from host before TXE_N asserts low.
   Without it, the FIFO bus stays idle.

### C / Host Side
1. **`FT_ReadPipe` works on macOS**, not `FT_ReadPipeEx` (returns FT_INVALID_PARAMETER).
2. **`FT_SetStreamPipe(handle, FALSE, FALSE, 0x82, chunk_size)`** must be called before
   any reads. This activates TXE_N on the FIFO bus.
3. **`FT_Create` requires the FTDI D3XX driver .dmg** installed on macOS. Without it,
   returns FT_DEVICE_NOT_OPENED (status 3). PyD3XX's bundled dylib alone is not enough.
4. **D3XX library version**: `/usr/local/lib/libftd3xx.dylib` from the FTDI .dmg.
   Must allow in System Settings → Privacy & Security after first run.
5. **USB cable must be data-capable** (not charge-only).
6. **Unplug/replug USB after reflashing** FPGA — FT600 state is stale after JTAG flash.

## Open Issue: Async FIFO CDC Duplicates

The async FIFO path (32 MHz write → 100 MHz read) produces ~5 duplicate
words per 10 MB. Both the settle-after-accept and the look-ahead bypass
produce the same error pattern, ruling out read-side timing as the cause.

**Error signature:** `got = expected - 1` — the FIFO outputs the same word
twice. The host counter advances past the duplicate, making the next valid
word appear as expected+1 but the duplicate appears as expected-1.

**Likely root cause:** The dual-clock memory (`reg [W-1:0] mem [0:D-1]`)
is written at 32 MHz and read combinationally at 100 MHz. When a 32 MHz
write edge lands near a 100 MHz read, the combinational read can capture
a transitioning value. Since the write side writes `counter` and `counter+1`
to adjacent addresses, a corrupted read could yield the previous word
(duplicate) rather than the current word.

**Evidence:**
- Direct counter (no FIFO, single clock domain): 0 errors, 35 MB/s
- Same counter through async FIFO: ~5 errors/10 MB at 22 MB/s
- Both settle and look-ahead produce identical error rates
- Errors are always duplicates (off by -1), never skips or random corruption

**Path forward:**
1. Use ECP5 block RAM (DP16KD) with true dual-port synchronous reads
   instead of LUT-based distributed RAM with async reads. Block RAM
   naturally registers outputs, eliminating the CDC read hazard.
2. Or: Add a registered read stage on the rclk side of the FIFO — read
   `mem[rptr]` into a register on rclk, then drive data_out from the
   register. This adds 1 cycle of latency but eliminates async reads
   across clock domains.
3. Or: Use a vendor-provided FIFO primitive (e.g., ECP5 FIFO16DC) that
   handles CDC internally with proper synchronization.
