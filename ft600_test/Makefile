# =============================================================================
# ft600_test/Makefile  —  Standalone FT600 counter-streamer test
#
# Targets:
#   make synth   — build bitstream
#   make prog    — load to SRAM via openFPGALoader (volatile)
#   make flash   — write to SPI flash via DirtyJTAG (persistent)
#   make clean   — remove build artefacts
# =============================================================================

OSS_CAD  := /opt/oss-cad-suite/bin
YOSYS    := $(OSS_CAD)/yosys
NEXTPNR  := $(OSS_CAD)/nextpnr-ecp5
ECPPACK  := $(OSS_CAD)/ecppack

DEVICE   := 25k
PACKAGE  := CABGA256
SPEED    := 7

SRCS     := top_ft600_test.v ft600_streamer.v
LPF      := ft600_test.lpf
JSON     := ft600_test.json
CFG      := ft600_test.cfg
BIT      := ft600_test.bit

.PHONY: synth prog flash clean

synth: $(BIT)
	@echo "FT600 test bitstream ready: $(BIT)"

$(JSON): $(SRCS)
	$(YOSYS) \
	  -p "read_verilog -sv $(SRCS); \
	      synth_ecp5 -top top -json $@"

$(CFG): $(JSON) $(LPF)
	$(NEXTPNR) \
	  --$(DEVICE) \
	  --package $(PACKAGE) \
	  --speed $(SPEED) \
	  --lpf $(LPF) \
	  --json $(JSON) \
	  --textcfg $@

$(BIT): $(CFG)
	$(ECPPACK) --compress $(CFG) $@

prog: $(BIT)
	$(OSS_CAD)/openFPGALoader $(BIT)

flash: $(BIT)
	$(OSS_CAD)/openFPGALoader -c dirtyJtag --fpga-part LFE5U-25F -f $(BIT)

clean:
	rm -f $(JSON) $(CFG) $(BIT)
