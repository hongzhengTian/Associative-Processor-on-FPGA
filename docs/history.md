# History

## Version 1 (2022)

The first version was a master's project: an associative processor built
from scratch in Verilog for a Xilinx Virtex-7 (xc7vx690t) board with DDR3
memory. It is preserved under the git tag **`v1-legacy`**, together with an
open-source simulation of it (Icarus Verilog, with behavioral models of the
Xilinx memory controller and FIFOs).

Its main parts were:

* three 16-word x 8-bit matrices (A, B, R) plus carry and flag columns, built
  from flip-flops, each loadable row by row or column by column;
* arithmetic (ADD, SUB, two's complement, ABS) done the classical way for
  content-addressable memories: per bit, several passes that compare masked
  keys to set tags and then write the tagged words;
* a custom 30-bit instruction set and a large hand-written controller state
  machine that also fetched instructions, handled jumps and interrupts, and
  saved the array context to memory on an interrupt;
* instruction and data caches in front of a DDR3 memory controller (Xilinx
  MIG), filled through a custom handshake.

## Why version 2 is a redesign

Building v1 taught most of what v2 does differently:

| v1 | v2 |
|---|---|
| The AP also had to be the processor: fetch, branches, interrupts in one state machine | A standard RISC-V core does all of that; the AP unit only executes AP instructions |
| Custom instruction set and assembler | RISC-V custom-1 instructions, written in C through `ap.h` and the stock GCC toolchain |
| Compare/write passes per bit, as in CAM-based designs | One full adder per row per bit step; tags, searches and reductions kept |
| Flip-flop matrices, 16 x 8 | Bit-plane storage in LUT RAM, parameterized up to 65536 x 2048 |
| Separate A/B/R matrices | One array; any bit range is an operand ("fields") |
| Context switch of the whole array on interrupts | AP state belongs to the program; interrupts stay in the CPU |
| DDR3 behind custom cache logic whose handshake depended on the memory controller's exact timing | Explicit scratchpad plus a DMA engine on standard AXI4 |
| Vendor project files and generated output in the repository | Plain sources, open-source simulation and synthesis, CI |
| One directed test program, checked against expected output prepared in a spreadsheet | Self-checking random tests against a reference model, and C test programs |

What carried over is the idea: a memory that computes on all of its words at
once, with per-word tags to select, search and reduce.

## Version 2

The current design. See the [README](../README.md) and
[architecture.md](architecture.md).
