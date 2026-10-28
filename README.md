# Associative Processor on FPGA

[![ci](https://github.com/hongzhengTian/Associative-Processor-on-FPGA/actions/workflows/ci.yml/badge.svg)](https://github.com/hongzhengTian/Associative-Processor-on-FPGA/actions/workflows/ci.yml)

An associative processing (AP) unit for RISC-V, written in SystemVerilog. The
AP unit holds an array of words and executes arithmetic, compares, searches
and reductions on **all words at once**, bit-serially: an operation on 16-bit
fields takes 17 cycles whether the array has 256 rows or 4096. A PicoRV32
core runs the program and issues AP instructions as RISC-V custom
instructions, written in plain C.

![SoC block diagram](docs/img/soc.svg)

## Highlights

* **Word-parallel, bit-serial array.** Data is stored in bit planes; every
  row has a one-bit ALU with carry, flag and tag. Array size is a parameter
  (ROWS up to 65536, COLS up to 2048).
* **Associative features.** Per-row tags make every operation conditional;
  compares chain with AND/OR; COUNT, FIRST and MAX/MIN turn parallel results
  back into scalars in one instruction.
* **Fields instead of registers.** An operand is any bit range of the rows,
  1 to 32 bits long, so data layout is as flexible as a C struct.
* **RISC-V integration.** 43 operations in the custom-1 opcode space, issued
  through PicoRV32's co-processor interface and wrapped by a small C header.
  No compiler changes.
* **Data movement.** A 32 x 32 transpose buffer moves 32 words per
  instruction, and an AXI4 DMA engine streams arrays from external memory
  (HBM/DDR on boards such as the Alveo U280).
* **Open tool flow.** Verilator simulation, RISC-V GCC, Yosys synthesis for
  UltraScale+, all in CI.

## Results

Cycle counts from the simulated SoC with a 256 x 128 array (`make test`):

| Example | CPU loop | AP | |
|---|---:|---:|---|
| vector add, 256 elements | 12,305 | 34 | 1,404 including DMA in and out |
| search: key == k, fetch the matching values | 7,783 | 252 | |
| search: range on two fields, count | 10,589 | 117 | |
| histogram of 256 bytes, 16 bins | 14,349 | 941 | |
| top 8 of 256 values | 98,957 | 722 | |

The CPU baseline is PicoRV32, a small multi-cycle core; the AP numbers are
the interesting part. They grow with the field width and the number of
instructions, not with the number of rows.

Synthesis with Yosys for UltraScale+ (pre-implementation estimates, see
[docs/synthesis.md](docs/synthesis.md)):

| target | LUT | LUTRAM | FF | CARRY | MUXF | BRAM | URAM | DSP |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| AP unit, 256 x 128 | 15440 | 1024 | 2175 | 1118 | 1333 | 0 | 0 | 0 |
| SoC (PicoRV32 + AP unit 256 x 128 + 64 KiB RAM) | 17777 | 1030 | 3226 | 1288 | 1999 | 16 | 0 | 0 |

## Quick start

Requirements: Verilator 5, a RISC-V GCC (`riscv64-unknown-elf-gcc`), Python 3,
and Yosys for synthesis. On Ubuntu 24.04:

```bash
sudo apt-get install verilator gcc-riscv64-unknown-elf yosys
```

```bash
make test                        # build the software, run all simulations
make -C sim run PROG=topk        # run one program on the SoC
make synth                       # Yosys synthesis and resource summary
```

A program looks like this:

```c
#include "ap.h"

const ap_field_t key = AP_FIELD(0, 12), value = AP_FIELD(12, 16);

ap_dma_config(table_addr, n);         // one record per row
ap_dma_load(key, 0);
ap_tag_range(0, n);
ap_cmps(AP_GEU, AP_TAND, key, 100);   // 100 <= key < 200, in every row at once
ap_cmps(AP_LTU, AP_TAND, key, 200);
uint32_t hits  = ap_count();
uint32_t first = ap_first();
uint32_t best  = ap_maxu(value);      // largest value among the hits
```

## Documentation

| Document | Content |
|---|---|
| [docs/architecture.md](docs/architecture.md) | How the SoC and the AP unit work, with diagrams |
| [docs/isa.md](docs/isa.md) | Instruction set reference |
| [docs/programming.md](docs/programming.md) | Programming guide and examples |
| [docs/verification.md](docs/verification.md) | Test strategy and how to run the tests |
| [docs/synthesis.md](docs/synthesis.md) | Synthesis results, FPGA notes, Alveo U280 integration |
| [docs/history.md](docs/history.md) | Version 1 and why it was redesigned |

## Repository layout

```
rtl/ap/            AP unit: core, bit-plane memory, row ALU, DMA
rtl/soc/           SoC: PicoRV32 + AP unit + RAM, PCPI adapter
third_party/       PicoRV32 (ISC license)
sw/include/        ap.h (AP instructions in C), runtime.h
sw/common/         start-up code, linker script, runtime, array helpers
sw/tests/          instruction-set test, smoke test
sw/examples/       vector_add, search, histogram, topk
sim/               Verilator testbenches, AXI memory model
syn/               Yosys synthesis scripts
docs/              documentation and figures
```

## History

This project started as a master's project with a self-designed instruction
set on a Virtex-7 board. Version 2 is a complete redesign; the first version is
kept under the git tag `v1-legacy`. See [docs/history.md](docs/history.md).

## License

MIT, see [LICENSE](LICENSE). PicoRV32 is under the ISC license, see
`third_party/picorv32/COPYING`.
