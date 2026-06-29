YOSYS   := yosys
NEXTPNR := nextpnr-ecp5
ECPPACK := ecppack

DESIGN  := top
LPF     := blink.lpf

JSON    := $(DESIGN).json
CFG     := $(DESIGN).config
BIT     := $(DESIGN).bit

.PHONY: all prog clean

all: $(BIT)

$(JSON): $(DESIGN).v
	$(YOSYS) -p "read_verilog $(DESIGN).v; synth_ecp5 -top $(DESIGN) -json $@"

$(CFG): $(JSON) $(LPF)
	$(NEXTPNR) --25k --package CABGA256 --json $(JSON) --lpf $(LPF) --textcfg $@

$(BIT): $(CFG)
	$(ECPPACK) --spimode fast-read $(CFG) $@

prog: $(BIT)
	openFPGALoader -c dirtyJtag --fpga-part LFE5U-25F -f $(BIT)

clean:
	rm -f $(JSON) $(CFG) $(BIT)
