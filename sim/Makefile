# Verilator simulations.
#
#   make              unit test of ap_core + all programs on the SoC + Zynq top
#   make ap_core      ap_core random test, several array shapes and seeds
#   make soc          every program in sw/ on the SoC, with and without
#                     AXI back-pressure
#   make zynq         the board programs through the Zynq top level and its
#                     host interface, array shape of the board build (512x64)
#   make host         the Zynq host program: Cortex-A9 build against stub BSP
#                     headers, and its benchmark comparison run natively
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
ZYNQ_RTL  := $(SOC_RTL) $(ROOT)/rtl/zynq/ap_host_bridge.sv $(ROOT)/rtl/zynq/ap_zynq.v

# shape of the Zynq board build and the programs the board host runs
ZYNQ_ROWS  ?= 512
ZYNQ_COLS  ?= 64
ZYNQ_PROGS := hello ap_isa_test vector_add search histogram topk scaling

# ap_core shapes "ROWSxCOLS" and seeds of the unit test
CORE_SHAPES ?= 32x8 64x48 96x40 128x128 256x16
CORE_SEEDS  ?= 1 2 3

PROGRAMS  := $(basename $(notdir $(wildcard $(ROOT)/sw/tests/*.c $(ROOT)/sw/examples/*.c)))

.PHONY: all ap_core soc zynq host run lint firmware clean
all: ap_core soc zynq host

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

# ------------------------------------------------------------- Zynq top
ZYNQ_BIN := $(BUILD)/zynq_$(ZYNQ_ROWS)x$(ZYNQ_COLS)/Vtb_ap_zynq

$(ZYNQ_BIN): $(ZYNQ_RTL) common/axi_mem_model.sv zynq/tb_ap_zynq.sv $(VLT)
	mkdir -p $(dir $@)
	$(VERILATOR) $(VFLAGS) -Mdir $(dir $@) --top-module tb_ap_zynq \
	    -GROWS=$(ZYNQ_ROWS) -GCOLS=$(ZYNQ_COLS) \
	    $(VLT) $(ZYNQ_RTL) common/axi_mem_model.sv zynq/tb_ap_zynq.sv

zynq: $(ZYNQ_BIN) firmware
	@set -e; for p in $(ZYNQ_PROGS); do for stall in 0 30; do \
	    log=$(dir $(ZYNQ_BIN))$$p.stall$$stall.log; \
	    $(ZYNQ_BIN) +firmware=$(ROOT)/sw/build/$$p.hex +axi_stall=$$stall > $$log; \
	    grep -q '^PASS' $$log || { cat $$log; echo "FAIL zynq $$p (axi_stall=$$stall)"; exit 1; }; \
	    echo "PASS zynq $$p (axi_stall=$$stall, $$(grep -o '\[tb_ap_zynq\] [0-9]* cycles' $$log | cut -d' ' -f2-))"; \
	done; done

# ------------------------------------------------------------ Zynq host
ARM_CC    ?= arm-none-eabi-gcc
HOST_SRC  := $(ROOT)/fpga/zynq/host
HOST_INC  := -Izynq/bsp -I$(HOST_SRC) -I$(ROOT)/sw/include

host: $(ZYNQ_BIN) firmware
	mkdir -p $(BUILD)/host
	$(ARM_CC) -mcpu=cortex-a9 -mfpu=neon -mfloat-abi=hard -O2 -Wall -Wextra -Werror \
	    $(HOST_INC) -c $(HOST_SRC)/main.c -o $(BUILD)/host/main_a9.o
	$(CC) -O2 -Wall -Wno-unused-function $(HOST_INC) zynq/host_check.c -o $(BUILD)/host/host_check
	$(ZYNQ_BIN) +firmware=$(ROOT)/sw/build/scaling.hex > $(BUILD)/host/scaling.log
	$(BUILD)/host/host_check $(BUILD)/host/scaling.log

run: $(SOC_BIN) firmware
	$(SOC_BIN) +firmware=$(ROOT)/sw/build/$(PROG).hex $(PLUSARGS)

lint:
	$(VERILATOR) --lint-only -Wall --top-module ap_soc $(VLT) $(SOC_RTL)
	$(VERILATOR) --lint-only -Wall --top-module ap_zynq $(VLT) $(ZYNQ_RTL)

clean:
	rm -rf $(BUILD)
