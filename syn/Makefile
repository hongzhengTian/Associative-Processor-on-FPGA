# Synthesis with Yosys for Xilinx UltraScale+ (the family of the Alveo U280).
#
#   make            synthesize the targets below, then write build/summary.md
#   make unit_256x128 / unit_1024x64 / soc   one target
#   make zynq       the Zynq top level for 7-series (XC7Z020 board build,
#                   512 x 64 array); not part of 'make'
#
# Yosys maps to UltraScale+ cells but does not place and route; the numbers
# are pre-implementation estimates (see docs/synthesis.md).

YOSYS  ?= yosys
PYTHON ?= python3
BUILD  ?= build
FAMILY ?= xcup

ROOT    := ..
AP_RTL  := $(addprefix $(ROOT)/rtl/ap/, ap_pkg.sv ap_plane_mem.sv ap_row_alu.sv \
                                        ap_core.sv ap_dma.sv ap_unit.sv)
SOC_RTL := $(AP_RTL) $(addprefix $(ROOT)/rtl/soc/, ap_pcpi.sv soc_ram.sv ap_soc.sv)
PICO    := $(ROOT)/third_party/picorv32/picorv32.v
ZYNQ_RTL := $(SOC_RTL) $(ROOT)/rtl/zynq/ap_host_bridge.sv
ZYNQ_ROWS ?= 512
ZYNQ_COLS ?= 64

UNITS   ?= 256x128
TARGETS := $(addprefix unit_,$(UNITS)) soc

.PHONY: all summary zynq $(TARGETS) clean
all: summary

summary: $(addprefix $(BUILD)/,$(addsuffix .log,$(TARGETS)))
	$(PYTHON) report.py $^ > $(BUILD)/summary.md
	cat $(BUILD)/summary.md

$(TARGETS): %: $(BUILD)/%.log

# Each log keeps only the final cell statistics (the full Yosys log of a
# large array runs into hundreds of megabytes).
$(BUILD)/unit_%.log: $(AP_RTL) | $(BUILD)
	$(YOSYS) -q -p "read_verilog -sv -DSYNTHESIS $(AP_RTL); \
	    chparam -set ROWS $(word 1,$(subst x, ,$*)) -set COLS $(word 2,$(subst x, ,$*)) ap_unit; \
	    synth_xilinx -family $(FAMILY) -top ap_unit -flatten; tee -q -o $@.tmp stat"
	mv $@.tmp $@

$(BUILD)/soc.log: $(SOC_RTL) $(PICO) | $(BUILD)
	$(YOSYS) -q -p "read_verilog -DSYNTHESIS $(PICO); \
	    read_verilog -sv -DSYNTHESIS $(SOC_RTL); \
	    synth_xilinx -family $(FAMILY) -top ap_soc -flatten; tee -q -o $@.tmp stat"
	mv $@.tmp $@

zynq: $(BUILD)/zynq.log
	$(PYTHON) report.py $< > $(BUILD)/zynq.md
	cat $(BUILD)/zynq.md

$(BUILD)/zynq.log: $(ZYNQ_RTL) $(ROOT)/rtl/zynq/ap_zynq.v $(PICO) | $(BUILD)
	$(YOSYS) -q -p "read_verilog -DSYNTHESIS $(PICO) $(ROOT)/rtl/zynq/ap_zynq.v; \
	    read_verilog -sv -DSYNTHESIS $(ZYNQ_RTL); \
	    chparam -set ROWS $(ZYNQ_ROWS) -set COLS $(ZYNQ_COLS) ap_zynq; \
	    synth_xilinx -family xc7 -top ap_zynq -flatten; tee -q -o $@.tmp stat"
	mv $@.tmp $@

$(BUILD):
	mkdir -p $@

clean:
	rm -rf $(BUILD)
