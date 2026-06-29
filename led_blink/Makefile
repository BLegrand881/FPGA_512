OSS_CAD := /opt/oss-cad-suite/bin
YOSYS   := $(OSS_CAD)/yosys
NEXTPNR := $(OSS_CAD)/nextpnr-ecp5
ECPPACK := $(OSS_CAD)/ecppack

DESIGN  := top
DEVICE  := um5g-85k
PACKAGE := CABGA381
SPEED   := 8
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
	$(OSS_CAD)/openFPGALoader -b ecp5_evn $(BIT)

clean:
	rm -f $(JSON) $(CFG) $(BIT)
