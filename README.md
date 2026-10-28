# Associative Processor on FPGA

A Verilog implementation of an associative processor (AP): a content-addressable
memory array that computes in place, bit-serially, on all of its words at
once. Programs and data live in an external DDR3 memory and are streamed in
through instruction and data caches. The design targets a Xilinx Virtex-7
(`xc7vx690tffg1761-2`) using the Xilinx MIG DDR3 controller.

## Architecture

| Block | Files | Role |
|---|---|---|
| Top level | `rtl/AP_top.v` | Connects everything below |
| Controller | `rtl/AP_ctrl.v` | Decodes instructions and sequences the compare/write passes of each arithmetic operation; saves/restores context on interrupts |
| CAM array | `rtl/CAM.v`, `rtl/cell_A.v`, `rtl/cell_B.v`, `rtl/cell_R.v`, `rtl/cell_C.v`, `rtl/cell_F.v`, `rtl/tag.v` | Matrices A, B and R (`DATA_DEPTH` words of `DATA_WIDTH` bits, accessible row by row or column by column), the carry (C) and flag (F) columns, and the tag register |
| Program counter | `rtl/Program_Counter.v` | Next-instruction address, jumps, return from interrupt |
| Instruction cache | `rtl/ins_cache.v`, `rtl/sm_ins_cache.v`, `rtl/ALU_ins_cache.v` | Fetches instruction blocks from DDR |
| Data cache | `rtl/data_cache.v`, `rtl/sm_data_cache.v`, `rtl/ALU_data_cache.v` | Loads/stores 16-word data blocks, flushes on interrupt |
| Interrupt stack | `rtl/int_stack.v` | Holds the controller state during an interrupt |
| DDR interface | `rtl/ddr3_interface_top.v`, `rtl/DDR_cache_interface.v`, `rtl/ddr_controller.v` | Loads the program and data into DDR at start-up, then serves cache requests through the MIG user interface |
| Xilinx IP | `ip/` | MIG 7-series DDR3 controller and two independent-clock FIFOs (DDR → caches) |

The core runs on `sys_clk_i_1` (100 MHz). The DDR side runs on `sys_clk_i_2`
(200 MHz, also the MIG reference clock).

## Instruction set

Instructions are 30 bits: `opcode[29:26] | cam_addr[25:18] | matrix[17:16] | mem_addr[15:0]`.
`matrix` selects `M_A` = 01, `M_B` = 10 or `M_R` = 11.

| Opcode | Mnemonic | Operation |
|---|---|---|
| 0001 | `RESET` | Clear the array |
| 0100 | `LOADRBR row M mem` | Load a word from memory into a row of matrix M |
| 0101 | `LOADCBC col M mem` | Load a bit column of matrix M from memory |
| 0110 | `STORERBR row M mem` | Store a row of matrix M to memory |
| 0111 | `STORECBC col M mem` | Store a bit column of matrix M to memory |
| 1000 | `COPY M1 M2` | Copy one matrix into another |
| 1001 | `ADD` | Add, word-parallel, on matrices A and B |
| 1010 | `SUB` | Subtract |
| 1011 | `TSC` | Two's complement |
| 1100 | `ABS` | Absolute value |
| 1101 | `PRINT mem` | Output a memory word on `data_print` |
| 0010 | `RET` | Return from the interrupt routine |
| 1110 | `STOP` | Halt (`finish_flag`) |

`tools/compiler.m` (MATLAB) assembles `ISA.txt` / `ISA_interrupt.txt` into the
binary files that the testbench loads.

## Repository layout

```
rtl/            synthesizable Verilog
ip/             Xilinx IP configurations (.xci, MIG .prj) only, no generated output
constraints/    AP_top.xdc: clocks and pins used by the project
                reference_ports_exported.xdc: full constraint export from an
                implemented run, kept for reference, not part of the project
sim/tb/         testbench (AP_top_tb.v) and Vivado waveform layout
sim/programs/   test program, interrupt routine, input data, expected output
sim/models/     behavioral stand-ins for the Xilinx IP, for open-source simulation
scripts/        create_project.tcl: rebuilds the Vivado project
tools/          compiler.m (assembler), referenceAnswer.xlsx (derivation of data_ref.txt)
```

## Simulation with open-source tools

Needs only [Icarus Verilog](https://steveicarus.github.io/iverilog/) (tested with 12.0):

```bash
make -C sim          # compile, run, compare against sim/programs/data_ref.txt
make -C sim wave     # also dump sim/build/AP_top_tb.vcd for GTKWave
```

The test program loads 128 bytes, runs ADD, SUB, TSC and ABS on them and
prints all 128 results. An interrupt arrives part way through and runs a
second routine. The run takes about 10 seconds and ends with
`PASS: all 128 results match programs/data_ref.txt`. CI runs the same check
on every push.

The Xilinx IP cannot be simulated outside Vivado, so `sim/models/` replaces it:

* `mig_7series_0.v` models the MIG user interface (app_* signals) with the
  DDR3 memory inside it.
* `async_fifo_model.v` models the two FIFO Generator instances.
* `ddr3_board_stubs.v` provides empty `ddr3_model` and `WireDelay` modules so
  the unchanged testbench compiles.
* `sim_watchdog.v` stops a run that hangs.

The models were calibrated against the original Vivado 2018.3 run (MIG with
the Micron DDR3 model): the results are bit-identical, and the DDR traffic of
each phase lines up with that run to within 1%.

**Known limitation: the DDR handshake depends on MIG timing.**
`ddr_controller.v` and `DDR_cache_interface.v` work only with the cycle-level
behavior of the real MIG, in two places:

* Each write phase stores one padding word first. `data_cache` and the `+1`
  burst lengths in `DDR_cache_interface` expect exactly one.
* In a runtime store, `ddr_controller` issues one write command fewer than it
  has data beats. The missing command comes from MIG holding `app_rdy` low
  for a couple of cycles at the start of the burst.

The model reproduces both effects on purpose; its header explains how.
Changing its timing parameters (for example with `PLUSARGS=+mig_row_switch=4`)
can make the design hang or produce wrong results. Making these two blocks
independent of controller timing would be a worthwhile improvement.

## Vivado flow

```bash
vivado -mode batch -source scripts/create_project.tcl
```

This creates `vivado/AP_5v.xpr` with the RTL, the IP (output products
generated from `ip/`), the constraints and the simulation set (testbench, the
Micron DDR3 model from the MIG example design, program files). Then run
behavioral simulation, synthesis or implementation from the GUI as usual.
The project was originally built with Vivado 2018.3; newer versions upgrade
the IP when the script runs. The Virtex-7 target needs a Vivado edition
that supports it.

## License

[MIT](LICENSE)
