OSS_CAD := /opt/oss-cad-suite/bin
YOSYS   := $(OSS_CAD)/yosys
NEXTPNR := $(OSS_CAD)/nextpnr-ecp5
ECPPACK := $(OSS_CAD)/ecppack

DESIGN  := top_bringup
LPF     := bringup.lpf
JSON    := $(DESIGN).json
CFG     := $(DESIGN).cfg
BIT     := $(DESIGN).bit

.PHONY: all flash clean

all: $(BIT)

$(JSON): $(DESIGN).v
	$(YOSYS) -p "read_verilog $(DESIGN).v; synth_ecp5 -top top -json $@"

$(CFG): $(JSON) $(LPF)
	$(NEXTPNR) --25k --package CABGA256 --speed 7 \
	  --lpf $(LPF) --json $(JSON) --textcfg $@

$(BIT): $(CFG)
	$(ECPPACK) --compress $(CFG) $@

flash: $(BIT)
	$(OSS_CAD)/openFPGALoader -c dirtyJtag --fpga-part LFE5U-25F -f $(BIT)

clean:
	rm -f $(JSON) $(CFG) $(BIT)
