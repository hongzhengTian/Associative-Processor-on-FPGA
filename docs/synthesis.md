# Synthesis and FPGA notes

## Yosys flow

```bash
make synth                          # all targets below, then a summary table
make -C syn unit_256x128            # one target
make -C syn UNITS="512x64 2048x32"  # other array shapes
```

CI synthesizes the 256 x 128 unit and the SoC. Larger shapes take much
longer in Yosys: 1024 x 64 needed about 20 minutes and 10 GB of memory.

`syn/Makefile` runs Yosys `synth_xilinx -family xcup` (UltraScale+, the
family of the Alveo U280) on:

* `unit_<ROWS>x<COLS>`: the AP unit (`ap_unit`: core and DMA) alone,
* `soc`: the whole SoC with PicoRV32 and the default 256 x 128 array.

Yosys maps the design to UltraScale+ cells (LUTs, flip-flops, LUT RAM, block
RAM, carry chains) but does not place and route it, so the numbers below are
estimates before implementation, and there is no timing result yet.

## Results

| target | LUT | LUTRAM | FF | CARRY | MUXF | BRAM | URAM | DSP |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| AP unit, 256 x 128 | 14487 | 1024 | 2175 | 124 | 471 | 0 | 0 | 0 |
| SoC (PicoRV32 + AP unit 256 x 128 + 64 KiB RAM) | 16082 | 1030 | 3226 | 294 | 548 | 16 | 0 | 0 |
| AP unit, 1024 x 64 (before the 128-row tag scans) | 31365 | 2048 | 4479 | 4190 | 3658 | 0 | 0 | 0 |

Columns are cell counts: LUT1-LUT6, LUT-RAM primitives (each covers several
LUTs), flip-flops, carry chains, wide multiplexers, block RAM, UltraRAM, DSP.

Observations:

* The array storage maps to LUT RAM, as intended (`ap_plane_mem`, one small
  memory per row and read port). The per-bit write mask that makes tagged
  writes cheap rules out block RAM and UltraRAM, which only have byte enables.
* Logic grows linearly with ROWS: every row has its own ALU, carry, flag and
  tag, and the write-data and tag-update multiplexers. The rest (decoder,
  sequencer, transpose buffer, DMA) is a small constant.
* The SoC adds about 1,600 LUTs for PicoRV32 and the bus logic. The 64 KiB
  program RAM takes 16 RAMB36 block RAMs. PicoRV32's multiplier is the
  iterative one, so no DSP slices are used.

## Notes for an implementation

**Mapping of the plane memory.** With COLS up to 64, a row fits in one
64-deep LUT RAM per read port; up to 256 it needs RAM128/RAM256 primitives.
Vivado normally packs the two read ports of a row into one multi-port LUT RAM
(RAM64M or RAM64M8); Yosys duplicates them, so its LUT RAM count is an upper
bound.

**Expected critical paths.** A bit step is single-cycle: asynchronous LUT RAM
read, row ALU, write back. Three paths are likely to set the clock frequency
in large arrays:

* the control signals that fan out to every row (bit index, operation,
  scalar bit), which the tools replicate;
* the OR over all rows in MAX/MIN (`mm_any`), which feeds back into every
  row's tag in the same cycle;
* the row multiplexer of element reads (ROWS : 1).

Each can be pipelined if needed (registered read data, a two-cycle step for
MAX/MIN, a staged read multiplexer) at the cost of a cycle per bit step or
per element access.

**Scaling.** Resource use is roughly proportional to ROWS: per row about 30
LUTs at 64 columns and 50 to 60 at 128 columns, plus the storage: two LUT RAM
primitives per row at 64 columns, four at 128. By that estimate 4096 rows of 64 columns need about 125k
LUTs, around a tenth of an Alveo U280. Measure with `make -C syn UNITS=...`
before choosing a shape.

## Boards

**Zynq-7000 (implemented).** `rtl/zynq/ap_zynq.v` wraps the SoC with a host
interface (`ap_host_bridge`): AXI4-Lite control registers, a port to load the
program RAM, a console FIFO, and an external-memory window with address
translation and bounds checking. `fpga/zynq/build.tcl` builds it in a block
design with the processing system; the ARM core runs the host program. See
[zynq.md](zynq.md). `make -C syn zynq` gives a Yosys estimate for the
XC7Z020 build:

| target | LUT | LUTRAM | FF | CARRY | MUXF | BRAM | URAM | DSP |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Zynq top level, 512 x 64 array, XC7Z020 (53,200 LUTs) | 22923 | 1084 | 4288 | 385 | 752 | 16 | 0 | 0 |

**Larger devices (Alveo U280 and similar).** The same host interface is what
a data-center card needs: wrap `ap_zynq` (or `ap_host_bridge` + `ap_soc`) as
an RTL kernel, drive the AXI4-Lite port from the host through the shell, and
connect the memory port to HBM or DDR4 through an AXI SmartConnect, which
converts width (HBM ports are 256 bits) and clock domain. The DMA engine
splits bursts at 4 KiB and uses only INCR bursts with one ID, so any AXI4
memory works. To use HBM bandwidth, the DMA data path would need to grow to
the port width so one beat fills several transpose-buffer words.
