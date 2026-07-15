OSS_CAD  := /opt/oss-cad-suite/bin
YOSYS    := $(OSS_CAD)/yosys
NEXTPNR  := $(OSS_CAD)/nextpnr-ecp5
ECPPACK  := $(OSS_CAD)/ecppack

DEVICE   := 25k
PACKAGE  := CABGA256
SPEED    := 7

SRCS     := top_adc_test.v \
            ../decoder/UWB_Serial_Handler.v \
            ../fpga/async_fifo.v \
            ../fpga/ft600_writer.v \
            ../fpga/spi_trigger.v
LPF      := adc_channel_test.lpf
JSON     := adc_channel_test.json
CFG      := adc_channel_test.cfg
BIT      := adc_channel_test.bit

HOST_CC  := cc
HOST_LIB := -L../ft600_test -lftd3xx -Wl,-rpath,../ft600_test

.PHONY: synth prog flash clean channel_test

synth: $(BIT)
	@echo "Bitstream ready: $(BIT)"

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

channel_test: channel_test.c ftd3xx.h
	$(HOST_CC) -o $@ channel_test.c $(HOST_LIB) -lm

clean:
	rm -f $(JSON) $(CFG) $(BIT) channel_test
