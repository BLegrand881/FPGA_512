# =============================================================================
# Makefile  —  ECP5-5G single-channel ADC serial loopback
# Toolchain: Yosys + nextpnr-ecp5 + ecppack
# Programmer: openFPGALoader  (or swap for ecpprog / fujprog)
# =============================================================================

DESIGN  := top
DEVICE  := um5g-85k          # LFE5UM5G-85F
PACKAGE := CABGA381
SPEED   := 8
LPF     := ecp5_eval.lpf

# Source files for synthesis — testbench excluded; top is the entry point.
# Yosys will only elaborate modules reachable from top so tb_adc_stream is
# safely ignored, but we keep the list explicit to avoid surprises.
SRCS := top.v ADC_Model_TB.v

# Intermediate / output artefacts
JSON := $(DESIGN).json
CFG  := $(DESIGN).cfg
BIT  := $(DESIGN).bit

# -----------------------------------------------------------------------------
.PHONY: all synth pnr pack prog sim clean

all: $(BIT)

# 1. Synthesis — Yosys with SystemVerilog mode for unpacked array ports
$(JSON): $(SRCS)
	yosys \
	  -p "read_verilog -sv $(SRCS); \
	      synth_ecp5 -top $(DESIGN) -json $@"

# 2. Place-and-route — nextpnr-ecp5
$(CFG): $(JSON) $(LPF)
	nextpnr-ecp5 \
	  --$(DEVICE) \
	  --package $(PACKAGE) \
	  --speed $(SPEED) \
	  --lpf $(LPF) \
	  --json $(JSON) \
	  --textcfg $@

# 3. Pack bitstream — ecppack
$(BIT): $(CFG)
	ecppack --compress $(CFG) $@

# 4. Program via openFPGALoader (USB JTAG on-board)
prog: $(BIT)
	openFPGALoader -b ecp5_eval $(BIT)

# 5. Simulation (iverilog) — runs the existing testbench
sim:
	iverilog -g2012 -o sim_tb ADC_Model_TB.v && vvp sim_tb

# Clean build artefacts
clean:
	rm -f $(JSON) $(CFG) $(BIT) sim_tb tb_adc_stream.vcd
