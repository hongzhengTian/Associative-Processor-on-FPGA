# Programming guide

Programs for the AP SoC are bare-metal C (RV32IM) built with a standard
`riscv64-unknown-elf-gcc`. AP instructions come from `sw/include/ap.h`, which
emits them with the assembler's `.insn` directive, so no compiler changes
are needed.

## Building and running

```bash
make -C sw                      # every program in sw/tests and sw/examples
make -C sim run PROG=search     # run one of them on the simulated SoC
```

A program starts at `_start` (`sw/common/start.S`), which sets up the stack
and `.bss` and calls `main()`. The return value of `main()` becomes the exit
code of the simulation; 0 means success. `sw/include/runtime.h` provides
`printf_()` (a small subset of `printf`), `rdcycle()` and `exit()`.

Memory map seen by programs:

| Address | Size | Content |
|---|---|---|
| 0x0000_0000 | 64 KiB | code, data, stack |
| 0x1000_0000 | | console (write a character) |
| 0x1000_0004 | | exit (write the exit code) |
| 0x8000_0000 | up to 2 GiB | external memory (`EXT_MEM_BASE`) |

## Planning fields

The array is ROWS rows of COLS bits (256 x 128 in the default build; read
them with `ap_rows()` and `ap_cols()`). Think of each row as one record and
lay out its fields like a struct, one bit range each:

```c
const ap_field_t key   = AP_FIELD(0, 12);   // bits 0..11
const ap_field_t value = AP_FIELD(12, 16);  // bits 12..27
const ap_field_t match = AP_FIELD(28, 1);   // a saved tag
const ap_field_t tmp   = AP_FIELD(32, 17);  // scratch, one bit wider for carries
```

Fields cost nothing to define and may overlap; sub-ranges of a field are
fields too. For example the high nibble of an 8-bit field `AP_FIELD(0, 8)`
is `AP_FIELD(4, 4)`, which `examples/histogram.c` uses to bin values with
one 4-bit compare.

Keep fields as short as the data allows: every instruction takes about one
cycle per bit of its destination (or compared) field.

## Moving data in and out

| Method | Use it for | Cost |
|---|---|---|
| `ap_write(f, row, v)` / `ap_read(f, row)` | single elements, results | L + 1 cycles each |
| `ap_put_array()` / `ap_get_array()` | arrays in CPU memory | 32 buffer accesses + L + 1 per 32 rows |
| `ap_dma_load()` / `ap_dma_store()` | arrays in external memory | about one cycle per element + L + 1 per 32 rows |

DMA takes 32-bit words, one per row:

```c
ap_dma_config((uint32_t)src, n);   // src in external memory, n elements
ap_dma_load(field, 0);             // rows 0 .. n-1
...
ap_dma_config((uint32_t)dst, n);
ap_dma_store(field, 0);
```

The functions return 0, or 1 if the memory reported an error.

## Tags: choosing rows

Arithmetic only changes rows whose tag is set. After reset every row is
tagged; `ap_tag_all()` restores that. Compares compute the tag and can be
chained:

```c
ap_tag_range(0, n);                       // only the rows in use
ap_cmps(AP_GEU, AP_TAND, key, 100);       // and key >= 100
ap_cmps(AP_LTU, AP_TAND, key, 200);       // and key < 200
ap_cmps(AP_EQ,  AP_TOR,  key, 0);         // or key == 0
uint32_t n_match = ap_count();
```

The tag can be stored in a 1-bit field and restored later with
`ap_tag_save()` / `ap_tag_load()`, which is how a program keeps several masks
around or walks through matches:

```c
ap_tag_save(match);
while ((r = ap_first()) != AP_NONE) {
    use(ap_read(value, r));
    ap_write(match, r, 0);     // done with this row
    ap_tag_load(match);
}
```

A data-dependent update (an "if" for every row) is a compare followed by the
operation:

```c
// x = (x < 0) ? -x : x   for signed 16-bit x, also available as ap_abs()
ap_cmps(AP_LT, AP_TSET, x, 0);
ap_neg(x, x);
ap_tag_all();
```

## Reductions

| Function | Result |
|---|---|
| `ap_count()` | number of tagged rows |
| `ap_first()` | lowest tagged row, `AP_NONE` if there is none |
| `ap_maxu(f)`, `ap_max(f)` | largest value among tagged rows; leaves only those rows tagged |
| `ap_minu(f)`, `ap_min(f)` | smallest value, likewise |

`ap_maxu()` plus `ap_first()` gives an arg-max. Repeating it while removing
the found row from a saved mask gives the K largest values, see
`examples/topk.c`.

## Things to keep in mind

* Results wrap modulo 2^L, L = destination length. Use a destination one bit
  longer than the operands to keep a carry.
* Sources shorter than the destination are zero-extended; signed operations
  expect operands of equal length.
* If a destination partly overlaps a source, it must not start above it.
* AP instructions block the CPU until they finish, so their results are
  ready for the next C statement.
* The `cond` and `mode` arguments of `ap_cmp()` / `ap_cmps()` must be
  constants: they are encoded into the instruction.

## Examples

| Program | Shows | Typical speed-up over the CPU loop (256 rows) |
|---|---|---|
| `examples/vector_add.c` | element-wise arithmetic, DMA | 360x for the add, 9x including DMA |
| `examples/search.c` | associative search, chained compares, walking matches | 33x and 100x |
| `examples/histogram.c` | field slicing, saved masks, COUNT | 19x |
| `examples/topk.c` | MAXU, FIRST, saved masks | 142x |
| `examples/scaling.c` | the benchmark of the Zynq build: AP cycles for growing n | |

The CPU baselines run on PicoRV32, a small multi-cycle core, so the ratios
overstate what a fast CPU would see; the absolute AP cycle counts are the
meaningful part. `tests/ap_isa_test.c` exercises every instruction and is a
compact reference for their use.
