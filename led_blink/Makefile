OSS_CAD := /opt/oss-cad-suite/bin
YOSYS   := $(OSS_CAD)/yosys
NEXTPNR := $(OSS_CAD)/nextpnr-ecp5
ECPPACK := $(OSS_CAD)/ecppack

DESIGN  := top
DEVICE  := 25k
PACKAGE := CABGA256
SPEED   := 6
LPF     := blink.lpf

JSON := $(DESIGN).json
CFG  := $(DESIGN).cfg
BIT  := $(DESIGN).bit

.PHONY: all prog clean

all: $(BIT)

$(JSON): $(DESIGN).v
	$(YOSYS) -p "read_verilog -sv $<; synth_ecp5 -top $(DESIGN) -json $@"

$(CFG): $(JSON) $(LPF)
	$(NEXTPNR) --$(DEVICE) --package $(PACKAGE) --speed $(SPEED) \
	           --lpf $(LPF) --json $(JSON) --textcfg $@

$(BIT): $(CFG)
	$(ECPPACK) --compress $(CFG) $@

prog: $(BIT)
	$(OSS_CAD)/openFPGALoader $(BIT)

clean:
	rm -f $(JSON) $(CFG) $(BIT)
