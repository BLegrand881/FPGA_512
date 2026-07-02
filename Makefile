# =============================================================================
# Makefile  —  ECP5 ADC receiver / UWB serial handler
# Toolchain: Yosys + nextpnr-ecp5 + ecppack
# Programmer: openFPGALoader  (or swap for ecpprog / fujprog)
#
# Targets:
#   make              — build bitstream for ECP5-5G eval board (default)
#   make synth-custom  — synthesise for custom board (LFE5U-25F BGA256)
#   make prog-custom   — load to SRAM via J2 JTAG (volatile)
#   make flash-custom  — write to SPI flash via Pico DirtyJTAG (persistent)
# =============================================================================

OSS_CAD  := /opt/oss-cad-suite/bin
YOSYS    := $(OSS_CAD)/yosys
NEXTPNR  := $(OSS_CAD)/nextpnr-ecp5
ECPPACK  := $(OSS_CAD)/ecppack
IVERILOG := iverilog

DESIGN  := top
DEVICE  := um5g-85k          # LFE5UM5G-85F (eval board)
PACKAGE := CABGA381
SPEED   := 8
LPF     := fpga/clock.lpf

# Custom board: LFE5U-25F-7BG256I
CUSTOM_DEVICE  := 25k
CUSTOM_PACKAGE := CABGA256
CUSTOM_SPEED   := 7
CUSTOM_LPF     := fpga/custom_board.lpf
CUSTOM_SRCS    := fpga/top_custom.v decoder/UWB_Serial_Handler.v
CUSTOM_JSON    := top_custom.json
CUSTOM_CFG     := top_custom.cfg
CUSTOM_BIT     := top_custom.bit

# Source files for synthesis — testbench excluded; top is the entry point.
# Yosys will only elaborate modules reachable from top so tb_adc_stream is
# safely ignored, but we keep the list explicit to avoid surprises.
SRCS := fpga/top.v decoder/UWB_Serial_Handler.v

# Intermediate / output artefacts
JSON := $(DESIGN).json
CFG  := $(DESIGN).cfg
BIT  := $(DESIGN).bit

# -----------------------------------------------------------------------------
.PHONY: all synth pnr pack prog sim clean synth-custom prog-custom flash-custom

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
	iverilog -g2012 -o sim_tb sim/ADC_Model_TB.v && vvp sim_tb

# 6. h5-capture-driven testbench
#    Step A: convert real ADC capture → hex stimulus + golden expected values
sim/stim_h5.hex: sine-all-5.h5 sim/extract_h5_stim.py
	conda run -n base python3 sim/extract_h5_stim.py

#    Step B: compile and run the testbench (depends on Step A)
sim-h5: sim/stim_h5.hex
	iverilog -g2012 -o sim_tb_h5 sim/tb_h5.v decoder/UWB_Serial_Handler.v
	vvp sim_tb_h5

#    Step C: decode serial CSV with receiver.py and compare against golden
sim/serial_sim_decoded.csv: sim/serial_sim.csv
	conda run -n base python3 receiver/receiver.py sim/serial_sim.csv \
	    --clk la_clk --data la_data_0 la_data_1 la_data_2 la_data_3 \
	    --lanes 0 1 2 3 --edge falling \
	    -o sim/serial_sim_decoded.csv

sim-compare: sim/serial_sim_decoded.csv sim/stim_h5.hex
	conda run -n base python3 sim/compare_sim.py

# -----------------------------------------------------------------------------
# Custom board build: LFE5U-25F-7BG256I (BGA256)
# Flash via J2 (JTAG header) using openFPGALoader.
# -----------------------------------------------------------------------------
$(CUSTOM_JSON): $(CUSTOM_SRCS)
	$(YOSYS) \
	  -p "read_verilog -sv $(CUSTOM_SRCS); \
	      synth_ecp5 -top top -json $@"

$(CUSTOM_CFG): $(CUSTOM_JSON) $(CUSTOM_LPF)
	$(NEXTPNR) \
	  --$(CUSTOM_DEVICE) \
	  --package $(CUSTOM_PACKAGE) \
	  --speed $(CUSTOM_SPEED) \
	  --lpf $(CUSTOM_LPF) \
	  --json $(CUSTOM_JSON) \
	  --textcfg $@

$(CUSTOM_BIT): $(CUSTOM_CFG)
	$(ECPPACK) --compress $(CUSTOM_CFG) $@

synth-custom: $(CUSTOM_BIT)
	@echo "Custom board bitstream ready: $(CUSTOM_BIT)"

# Program custom board via J2 JTAG — volatile SRAM load (lost on power cycle)
prog-custom: $(CUSTOM_BIT)
	$(OSS_CAD)/openFPGALoader $(CUSTOM_BIT)

# Flash custom board to SPI flash via Raspberry Pi Pico DirtyJTAG
# Bitstream survives power cycle; FPGA loads from flash on every boot.
# DirtyJTAG firmware: https://github.com/jeanthom/DirtyJTAG
flash-custom: $(CUSTOM_BIT)
	$(OSS_CAD)/openFPGALoader -c dirtyJtag --fpga-part LFE5U-25F -f $(CUSTOM_BIT)

# Clean build artefacts
clean:
	rm -f $(JSON) $(CFG) $(BIT) sim_tb tb_adc_stream.vcd \
	      sim_tb_h5 sim/tb_h5.vcd \
	      sim/stim_h5.hex sim/expected_ch*.hex \
	      $(CUSTOM_JSON) $(CUSTOM_CFG) $(CUSTOM_BIT)
