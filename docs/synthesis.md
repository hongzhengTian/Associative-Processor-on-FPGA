# Synthesis and FPGA notes

## Yosys flow

```bash
make synth                          # all targets below, then a summary table
make -C syn unit_256x128            # one target
make -C syn UNITS="512x64 2048x32"  # other array shapes
```

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
| AP unit, 256 x 128 | 15440 | 1024 | 2175 | 1118 | 1333 | 0 | 0 | 0 |

Columns are cell counts: LUT1-LUT6, LUT-RAM primitives (each covers several
LUTs), flip-flops, carry chains, wide multiplexers, block RAM, UltraRAM, DSP.

Observations:

* The array storage maps to LUT RAM, as intended (`ap_plane_mem`, one small
  memory per row and read port). The per-bit write mask that makes tagged
  writes cheap rules out block RAM and UltraRAM, which only have byte enables.
* Logic grows linearly with ROWS: every row has its own ALU, carry, flag and
  tag, and the write-data and tag-update multiplexers. The rest (decoder,
  sequencer, transpose buffer, DMA) is a small constant.

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

**Scaling.** Resource use is roughly proportional to ROWS x (per-row logic +
COLS / 64 LUT RAMs). The default build is small on purpose; for an Alveo-size
device, thousands of rows fit. Measure with `make -C syn UNITS=...` before
choosing a shape.

## Alveo U280 integration

The SoC has exactly the interfaces an Alveo design needs, but the board
integration itself is not part of this repository yet. A suggested path:

1. **Kernel wrapper.** Wrap `ap_soc` as an RTL kernel: an AXI4-Lite control
   port for start/reset/status, a way to load the program RAM (either an
   initialization file through the `INIT_FILE` parameter, or a second port
   on `soc_ram` written by the host), and the console/exit I/O port mapped to
   registers or a small FIFO.
2. **Memory.** Connect `m_axi` (AXI4, 32-bit data, 31-bit address) to an HBM
   pseudo-channel or DDR4 through an AXI SmartConnect, which handles width
   (HBM ports are 256 bits wide) and clock-domain conversion. The DMA engine
   splits bursts at 4 KiB boundaries and uses only INCR bursts with a single
   ID, so any AXI4 slave works.
3. **Clock and reset.** The design is a single synchronous clock domain with
   an active-low reset; drive it from a kernel clock. Start with a moderate
   clock and a small array, then scale.
4. **Wider DMA.** For bandwidth, widen the DMA data path to the HBM port width
   so one beat fills several transpose-buffer words.

Simulation already covers the AXI behaviour (random back-pressure, protocol
checks), so the remaining work is mostly the Vitis/XRT side and timing
closure.
