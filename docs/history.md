# History

## Version 1 (2022)

The first version was a master's project: an associative processor built
from scratch in Verilog for a Xilinx Virtex-7 (xc7vx690t) board with DDR3
memory, published as *FPGA Implementation of Associative Processors*, IEEE
Transactions on Circuits and Systems II: Express Briefs, vol. 70, no. 5,
pp. 1774-1778 ([IEEE Xplore](https://ieeexplore.ieee.org/document/10081044)).
It is preserved under the git tag **`v1-legacy`**, together with an
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

What carried over is the associative processing model of the paper: a CAM
array of words that is searched, updated and reduced in parallel, bit-serially,
with a tag per word. Version 2 changes how it is built and used, not what it
computes. The next section compares the AP core of both versions in detail.

## The AP core in v1 and v2

The RTL in `rtl/ap/` is a new implementation of the same model. It does not
reuse the v1 modules, but every part of the v1 array has a counterpart:

| v1 | v2 | Change |
|---|---|---|
| `CAM.v`, `cell_A.v`, `cell_B.v`, `cell_R.v`: three matrices A, B, R of flip-flops, each bit stored as Q and its complement Qb | `ap_plane_mem.sv`: one array of ROWS x COLS bits, stored as bit planes in LUT RAM | Operands are fields (any bit range of a word) instead of fixed matrices |
| `cell_C.v`: carry column | carry register per row in `ap_core.sv` | Same role |
| `cell_F.v`: flag column for the sign in TSC and ABS | flag register per row in `ap_core.sv` | Also holds the running result of compares |
| `tag.v`: AND of the match lines of A, B and C, cleared before every pass, internal only | tag register in `ap_core.sv` | Visible to programs: compares set, AND or OR it; it can be saved to a 1-bit field and loaded back |
| Row-by-row and column-by-column load and output logic in every matrix | 32 x 32 transpose buffer, element access, DMA | One path for data in and out |
| `AP_ctrl.v` (about 3,000 lines): instruction fetch, branches, interrupts, context save, cache handshake and the pass sequences | `ap_core.sv` (under 500 lines): decoding and the bit-serial sequence only | Fetch, branches and interrupts belong to the RISC-V core |

### One bit position: four passes become one cycle

v1 adds the classical way for a CAM, in place (B = A + B), with four passes
per bit position. In each pass the mask selects the current bit column and
the keys select one combination of the bits of A, B and the carry C. Every
word that matches is tagged, and then the tagged words are written:

| Pass | A B C matched | Written to the tagged words |
|---|---|---|
| 1 | 1 1 0 | B bit becomes 0, carry becomes 1 |
| 2 | 1 0 0 | B bit becomes 1 |
| 3 | 0 0 1 | B bit becomes 1, carry becomes 0 |
| 4 | 0 1 1 | B bit becomes 0 |

These are the four rows of the full-adder truth table in which B or the carry
changes. Their order ensures that a word changed by one pass does not match a
later one. Each pass is held for eight clock cycles and followed by a tag
reset, so one bit position takes about 37 cycles.

v2 gives every word a one-bit ALU (`ap_row_alu.sv`) that evaluates the whole
truth table at once: sum = x ^ y ^ c and the carry of a full adder. The tag is
the write enable of the bit-plane memory (`w_mask` of `ap_plane_mem.sv`), so
only tagged words are written, as before. One bit position takes one cycle,
and the result can go to a third field (d = a + b). Subtraction, negation and
absolute value change only the adder inputs (an inverted operand, an initial
carry of 1) instead of needing their own pass sequences.

Compares are bit-serial in both versions. In v1 the key of a pass is a single
bit compared with the selected column. In v2 the compare runs from the least
to the most significant bit and keeps its running result in the flag
register, which then sets, ANDs or ORs the tag.

### Other changes in the core

* **Fields instead of matrices.** In v1 operands had to be copied into A and
  B, and results appeared in B or R. In v2 any bit range of a word is a
  source or a destination, so no copies between matrices are needed.
* **Reductions are new.** COUNT, FIRST and MAX/MIN did not exist in v1.
  MAX/MIN is the classical associative maximum search: from the most
  significant bit down, if any tagged word has a 1 at that bit, the tagged
  words with a 0 there lose their tag.
* **Storage.** v1 used two flip-flops per bit (Q and Qb) and enable logic per
  cell, and was built as 16 words of 8 bits. v2 keeps one COLS x 1 LUT RAM per
  row, with the tag as its write enable. The array is parameterized up to
  65536 x 2048; the Zynq-7020 build uses 512 x 64.
* **Coding style.** v2 is synthesizable SystemVerilog that runs in Verilator
  and Yosys, and it is checked by random tests against a reference model.

### Not a pure CAM any more

Every step of v1 is a search for a key followed by a write, so its array could
in principle be built from real CAM cells. v2 adds a little logic per word and
merges the searches and writes of one bit position into one step. The array
then works more like a bit-serial SIMD array with a one-bit processing element
per word. On an FPGA this is a good trade: an FPGA has no CAM cells anyway
(v1 also built its array from flip-flops and logic), and a few LUTs per word
save more than 30 cycles per bit. Tags, searches, writes to tagged words only,
and reductions, the parts that define an associative processor, are
unchanged.

## Version 2

The current design. See the [README](../README.md) and
[architecture.md](architecture.md).
