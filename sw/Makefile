# Bare-metal programs for the AP SoC (RV32IM).
#
#   make            build every program in tests/ and examples/
#   make <name>     build build/<name>.{elf,hex,lst}

CROSS   ?= riscv64-unknown-elf-
CC      := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
OBJDUMP := $(CROSS)objdump
PYTHON  ?= python3

BUILD   ?= build
ARCH    := -march=rv32im_zicsr_zicntr -mabi=ilp32
CFLAGS  := $(ARCH) -O2 -g -Wall -Wextra -ffreestanding -fno-builtin -nostdlib \
           -fno-pic -Iinclude
LDFLAGS := $(ARCH) -nostdlib -nostartfiles -T common/link.ld -Wl,--gc-sections -Wl,--no-warn-rwx-segments
LIBS    := -lgcc

COMMON  := common/start.S common/runtime.c common/aplib.c
PROGS   := $(basename $(notdir $(wildcard tests/*.c examples/*.c)))

vpath %.c tests examples

.PHONY: all clean $(PROGS)
all: $(PROGS)

$(PROGS): %: $(BUILD)/%.hex

$(BUILD)/%.elf: %.c $(COMMON) common/link.ld $(wildcard include/*.h) | $(BUILD)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(COMMON) $< $(LIBS)

$(BUILD)/%.hex: $(BUILD)/%.elf
	$(OBJCOPY) -O binary $< $(BUILD)/$*.bin
	$(PYTHON) tools/bin2hex.py $(BUILD)/$*.bin $@
	$(OBJDUMP) -d $< > $(BUILD)/$*.lst

$(BUILD):
	mkdir -p $@

clean:
	rm -rf $(BUILD)

.PRECIOUS: $(BUILD)/%.elf
