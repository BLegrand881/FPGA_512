# =============================================================================
# Makefile  —  ECP5-5G single-channel ADC serial loopback
# Toolchain: Yosys + nextpnr-ecp5 + ecppack
# Programmer: openFPGALoader  (or swap for ecpprog / fujprog)
# =============================================================================

OSS_CAD  := /opt/oss-cad-suite/bin
YOSYS    := $(OSS_CAD)/yosys
NEXTPNR  := $(OSS_CAD)/nextpnr-ecp5
ECPPACK  := $(OSS_CAD)/ecppack
IVERILOG := iverilog

DESIGN  := top
DEVICE  := um5g-85k          # LFE5UM5G-85F
PACKAGE := CABGA381
SPEED   := 8
LPF     := fpga/clock.lpf

# Source files for synthesis — testbench excluded; top is the entry point.
# Yosys will only elaborate modules reachable from top so tb_adc_stream is
# safely ignored, but we keep the list explicit to avoid surprises.
SRCS := fpga/top.v fpga/ADC_Model_TB.v fpga/UWB_Serial_Handler.v

# Intermediate / output artefacts
JSON := $(DESIGN).json
CFG  := $(DESIGN).cfg
BIT  := $(DESIGN).bit

# -----------------------------------------------------------------------------
.PHONY: all synth pnr pack prog sim clean

all: $(BIT)

# 1. Synthesis — Yosys with SystemVerilog mode for unpacked array ports
$(JSON): $(SRCS)
	$(YOSYS) \
	  -p "read_verilog -sv $(SRCS); \
	      synth_ecp5 -top $(DESIGN) -json $@"

# 2. Place-and-route — nextpnr-ecp5
$(CFG): $(JSON) $(LPF)
	$(NEXTPNR) \
	  --$(DEVICE) \
	  --package $(PACKAGE) \
	  --speed $(SPEED) \
	  --lpf $(LPF) \
	  --json $(JSON) \
	  --textcfg $@

# 3. Pack bitstream — ecppack
$(BIT): $(CFG)
	$(ECPPACK) --compress $(CFG) $@

# 4. Program via openFPGALoader (USB JTAG on-board)
# Note: 'prog' depends on $(BIT) so it runs the full build chain automatically.
# Full flow in one command: make sim && make prog
prog: $(BIT)
	$(OSS_CAD)/openFPGALoader -b ecp5_evn $(BIT)

# 5. Simulation (iverilog) — runs the existing testbench
sim:
	iverilog -g2012 -o sim_tb fpga/ADC_Model_TB.v && vvp sim_tb

# Clean build artefacts
clean:
	rm -f $(JSON) $(CFG) $(BIT) sim_tb tb_adc_stream.vcd
