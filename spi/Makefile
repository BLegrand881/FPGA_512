# Toolchain binaries
YOSYS       := yosys
NEXTPNR     := nextpnr-ecp5
ECPPACK     := ecppack
PROGRAMMER  := openFPGALoader

# Design files
DESIGN      := top
LPF         := bringup.lpf

# Intermediate and output files
JSON        := $(DESIGN).json
CFG         := $(DESIGN).config
BIT         := $(DESIGN).bit

.PHONY: all prog clean

# Default target: builds the bitstream
all: $(BIT)

# 1. Synthesis (Yosys)
$(JSON): $(DESIGN).v
	$(YOSYS) -p "read_verilog $(DESIGN).v; synth_ecp5 -top $(DESIGN) -json $@"

# 2. Place & Route (nextpnr-ecp5 for LFE5U-25F CABGA256)
$(CFG): $(JSON) $(LPF)
	$(NEXTPNR) --25k --package CABGA256 --json $(JSON) --lpf $(LPF) --textcfg $@

# 3. Pack Bitstream (ecppack)
$(BIT): $(CFG)
	$(ECPPACK) --spimode fast-read $(CFG) $@

# 4. Program directly to SPI Flash using DirtyJTAG
prog: $(BIT)
	$(PROGRAMMER) -c dirtyJtag --fpga-part LFE5U-25F -f $(BIT)

# Windows-friendly cleanup rule
clean:
	@if exist $(JSON) del /f $(JSON)
	@if exist $(CFG) del /f $(CFG)
	@if exist $(BIT) del /f $(BIT)