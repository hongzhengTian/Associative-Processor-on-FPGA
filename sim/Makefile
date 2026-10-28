# Open-source simulation of AP_top with Icarus Verilog.
#
# The Xilinx IP (MIG DDR3 controller, two FIFO Generator FIFOs) is replaced by
# the behavioral models in models/, so no Vivado installation is needed.
#
#   make            compile, run the test program and check the result
#   make wave       same, and dump all signals to build/AP_top_tb.vcd
#   make clean      remove build/
#
# Extra run-time options can be passed with PLUSARGS, e.g.
#   make PLUSARGS=+timeout_ns=500000

IVERILOG ?= iverilog
VVP      ?= vvp
BUILD    ?= build
PLUSARGS ?=

RTL      := $(wildcard ../rtl/*.v)
MODELS   := $(wildcard models/*.v)
TB       := tb/AP_top_tb.v
PROGRAMS := $(wildcard programs/*.txt)

.PHONY: all sim wave clean

all: sim

$(BUILD)/AP_top_tb.vvp: $(RTL) $(MODELS) $(TB) | $(BUILD)
	$(IVERILOG) -g2005 -o $@ -s AP_top_tb -s sim_watchdog $(EXTRA_TOPS) $(MODELS) $(TB) $(RTL)

$(BUILD):
	mkdir -p $@

# The testbench reads its program and data files from the working directory.
sim: $(BUILD)/AP_top_tb.vvp $(PROGRAMS)
	cp $(PROGRAMS) $(BUILD)/
	rm -f $(BUILD)/AP_output.txt
	cd $(BUILD) && $(VVP) -n AP_top_tb.vvp $(PLUSARGS) | tee sim.log
	@grep -q "simulation success" $(BUILD)/sim.log || { echo "FAIL: testbench reported errors"; exit 1; }
	@cmp -s $(BUILD)/AP_output.txt programs/data_ref.txt || { echo "FAIL: AP_output.txt differs from programs/data_ref.txt"; exit 1; }
	@echo "PASS: all $$(wc -l < programs/data_ref.txt) results match programs/data_ref.txt"

wave: EXTRA_TOPS = -s vcd_dump
wave: MODELS += $(BUILD)/vcd_dump.v
wave: $(BUILD)/vcd_dump.v
	rm -f $(BUILD)/AP_top_tb.vvp
	$(MAKE) sim EXTRA_TOPS="$(EXTRA_TOPS)" MODELS="$(MODELS)"

$(BUILD)/vcd_dump.v: | $(BUILD)
	printf 'module vcd_dump; initial begin $$dumpfile("AP_top_tb.vcd"); $$dumpvars(0, AP_top_tb); end endmodule\n' > $@

clean:
	rm -rf $(BUILD)
