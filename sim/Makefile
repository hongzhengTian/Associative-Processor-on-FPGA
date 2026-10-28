# Verilator simulations.
#
#   make              unit test of ap_core + all programs on the SoC
#   make ap_core      ap_core random test, several array shapes and seeds
#   make soc          every program in sw/ on the SoC, with and without
#                     AXI back-pressure
#   make run PROG=x   run one program, e.g. PROG=search (PLUSARGS=... too)
#   make lint         Verilator -Wall lint of the RTL
#
# Array shape of the SoC: ROWS=256 COLS=128 by default.

VERILATOR ?= verilator
BUILD     ?= build
ROWS      ?= 256
COLS      ?= 128
NCMDS     ?= 3000
PLUSARGS  ?=

ROOT      := ..
VLT       := verilator.vlt
VFLAGS    := --binary --timing -Wall -j 0 --unroll-count 70000

CORE_RTL  := $(addprefix $(ROOT)/rtl/ap/, ap_pkg.sv ap_plane_mem.sv ap_row_alu.sv ap_core.sv)
AP_RTL    := $(CORE_RTL) $(addprefix $(ROOT)/rtl/ap/, ap_dma.sv ap_unit.sv)
SOC_RTL   := $(AP_RTL) $(addprefix $(ROOT)/rtl/soc/, ap_pcpi.sv soc_ram.sv ap_soc.sv) \
             $(ROOT)/third_party/picorv32/picorv32.v

# ap_core shapes "ROWSxCOLS" and seeds of the unit test
CORE_SHAPES ?= 32x8 64x48 96x40 128x128
CORE_SEEDS  ?= 1 2 3

PROGRAMS  := $(basename $(notdir $(wildcard $(ROOT)/sw/tests/*.c $(ROOT)/sw/examples/*.c)))

.PHONY: all ap_core soc run lint firmware clean
all: ap_core soc

# ------------------------------------------------------------- ap_core unit
CORE_BINS := $(foreach s,$(CORE_SHAPES),$(BUILD)/core_$(s)/Vtb_ap_core)

$(BUILD)/core_%/Vtb_ap_core: $(CORE_RTL) ap_core/tb_ap_core.sv $(VLT)
	mkdir -p $(dir $@)
	$(VERILATOR) $(VFLAGS) -Mdir $(BUILD)/core_$* --top-module tb_ap_core \
	    -GROWS=$(word 1,$(subst x, ,$*)) -GCOLS=$(word 2,$(subst x, ,$*)) \
	    $(VLT) $(CORE_RTL) ap_core/tb_ap_core.sv

ap_core: $(CORE_BINS)
	@set -e; for s in $(CORE_SHAPES); do for seed in $(CORE_SEEDS); do \
	    $(BUILD)/core_$$s/Vtb_ap_core +seed=$$seed +ncmds=$(NCMDS) > $(BUILD)/core_$$s/seed$$seed.log; \
	    grep -q '^PASS' $(BUILD)/core_$$s/seed$$seed.log || \
	        { cat $(BUILD)/core_$$s/seed$$seed.log; echo "FAIL ap_core $$s seed $$seed"; exit 1; }; \
	    grep 'commands' $(BUILD)/core_$$s/seed$$seed.log; \
	done; done

# ------------------------------------------------------------------- SoC
SOC_BIN := $(BUILD)/soc_$(ROWS)x$(COLS)/Vtb_soc

$(SOC_BIN): $(SOC_RTL) common/axi_mem_model.sv soc/tb_soc.sv $(VLT)
	mkdir -p $(dir $@)
	$(VERILATOR) $(VFLAGS) -Mdir $(dir $@) --top-module tb_soc \
	    -GROWS=$(ROWS) -GCOLS=$(COLS) \
	    $(VLT) $(SOC_RTL) common/axi_mem_model.sv soc/tb_soc.sv

firmware:
	$(MAKE) -C $(ROOT)/sw

soc: $(SOC_BIN) firmware
	@set -e; for p in $(PROGRAMS); do for stall in 0 30; do \
	    log=$(dir $(SOC_BIN))$$p.stall$$stall.log; \
	    $(SOC_BIN) +firmware=$(ROOT)/sw/build/$$p.hex +axi_stall=$$stall > $$log; \
	    grep -q '^PASS' $$log || { cat $$log; echo "FAIL $$p (axi_stall=$$stall)"; exit 1; }; \
	    echo "PASS $$p (axi_stall=$$stall, $$(grep -o 'after [0-9]* cycles' $$log))"; \
	done; done

run: $(SOC_BIN) firmware
	$(SOC_BIN) +firmware=$(ROOT)/sw/build/$(PROG).hex $(PLUSARGS)

lint:
	$(VERILATOR) --lint-only -Wall --top-module ap_soc $(VLT) $(SOC_RTL)

clean:
	rm -rf $(BUILD)
