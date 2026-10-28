# AP instruction set

The AP unit extends RV32IM with instructions in the **custom-1** major opcode
(`0101011`, 0x2B). PicoRV32's own custom instructions (IRQ handling) live in
custom-0, so the two never collide.

All AP instructions use the R-type format. `funct3` selects a group, `funct7`
the operation within it:

```
 31        25 24    20 19    15 14  12 11     7 6       0
+------------+--------+--------+------+--------+---------+
|   funct7   |  rs2   |  rs1   |funct3|   rd   | 0101011 |
+------------+--------+--------+------+--------+---------+
```

The CPU stalls until an instruction has finished, so its result can be used
by the very next instruction and AP state never changes behind software's
back. Instructions without a result leave `rd` unchanged (the C interface
uses `x0`). An encoding not listed here is illegal and raises PicoRV32's
illegal-instruction trap.

## Programming model

| State | Size | Description |
|---|---|---|
| array | ROWS x COLS bits | ROWS words ("rows") of COLS bits. Contents survive reset. |
| tag | ROWS bits | One bit per row. Arithmetic writes only tagged rows; compares compute the tag. All ones after reset. |
| xbuf | 32 x 32 bits | Transpose buffer between CPU words and 32 rows. Zero after reset. |
| DMA address, count | 2 x 32 bits | Set by `DMA_CFG`, used by `DMA_LOAD` / `DMA_STORE`. |

ROWS (a multiple of 32, up to 65536) and COLS (up to 2048) are synthesis
parameters; `SYS_INFO` reports them.

### Fields

Operands are **fields**: the same bit range in every row. A field descriptor
is 16 bits:

```
 15     11 10             0
+---------+----------------+
| len - 1 |     offset     |      len = 1..32 bits, offset = 0..2047
+---------+----------------+
```

`AP_FIELD(offset, len)` in `ap.h` builds one. Fields may sit anywhere and may
overlap. The low and high bits of a field are fields themselves, which is
often useful: the top four bits of an 8-bit field at offset 0 are
`AP_FIELD(4, 4)`.

Rules that apply to every instruction:

* An operation on a destination field of length L processes L bit positions.
  Source bits beyond a source field's own length read as 0 (zero extension);
  a source longer than L is cut to L bits.
* Columns at or beyond COLS read as 0 and are never written.
* Signed operations (`ABS`, signed compares, `MAX`, `MIN`) treat a field as a
  two's-complement number of its own length. Give signed operands the same
  length.
* A destination may coincide with a source. If it only partly overlaps a
  source, the destination must not start above that source (bits are
  processed LSB first); otherwise the result is undefined.

Register operands that carry two 16-bit values are written `{hi, lo}` below,
meaning `hi` in bits 31:16 and `lo` in bits 15:0 (`AP_PAIR(hi, lo)`).

### Timing

Times are AP clock cycles from issue to result inside the AP unit; PCPI adds
a few CPU cycles per instruction. L is the length of the field that is
iterated over.

| Kind | Cycles |
|---|---|
| tag set/clear/invert, xbuf word access, info | 1 |
| field operation, compare, MAX/MIN, element access, XPUT/XGET | L + 1 |
| ABS | L + 2 |
| TAG_RANGE, COUNT | ROWS/32 + 1 |
| FIRST | 1 + number of 32-row chunks up to the first tagged row |
| DMA | about one cycle per word plus L + 1 per 32 rows, plus memory latency |

The cost of a field operation does not depend on ROWS: one instruction
processes every row.

## funct3 = 0, 1: field arithmetic and logic

`rs1 = {a, d}`. For funct3 = 0, `rs2[15:0]` is a field b. For funct3 = 1 the
second operand is the 32-bit scalar `s = rs2`, the same for every row.
Only tagged rows are written.

| funct7 | Name | Operation | C (field b / scalar s) |
|---:|---|---|---|
| 0 | ADD | d = a + b | `ap_add(d, a, b)` / `ap_adds(d, a, s)` |
| 1 | SUB | d = a - b | `ap_sub` / `ap_subs` |
| 2 | AND | d = a & b | `ap_and` / `ap_ands` |
| 3 | OR | d = a \| b | `ap_or` / `ap_ors` |
| 4 | XOR | d = a ^ b | `ap_xor` / `ap_xors` |
| 5 | MOV | d = a | `ap_mov(d, a)` |
| 6 | NOT | d = ~a | `ap_not(d, a)` |
| 7 | NEG | d = -a | `ap_neg(d, a)` |
| 8 | ABS | d = \|a\|, a signed | `ap_abs(d, a)` |
| 9 | SET | d = b | `ap_copy(d, b)` / `ap_sets(d, s)` |
| 10 | RSUB | d = b - a | `ap_rsub` / `ap_rsubs` |

Results are taken modulo 2^L. MOV, NOT, NEG and ABS ignore the second
operand in both groups. In SET, a is ignored.

## funct3 = 2: compare

`rs1[15:0]` = field a (L = its length). `rs2[15:0]` = field b, or with the
scalar bit set, `rs2` = scalar s (its low L bits are used).

```
funct7 = { mode[1:0], scalar, cond[3:0] }
```

| cond | Name | Condition | | mode | Name | New tag |
|---:|---|---|---|---:|---|---|
| 0 | EQ | a == b | | 0 | SET | cond |
| 1 | NE | a != b | | 1 | AND | tag & cond |
| 2 | LTU | a < b unsigned | | 2 | OR | tag \| cond |
| 3 | LT | a < b signed | | 3 | | reserved |
| 4 | GEU | a >= b unsigned | | | | |
| 5 | GE | a >= b signed | | | | |
| 6 | GTU | a > b unsigned | | | | |
| 7 | GT | a > b signed | | | | |
| 8 | LEU | a <= b unsigned | | | | |
| 9 | LE | a <= b signed | | | | |

Every row is compared; the mode decides how the outcome combines with the
current tag. C: `ap_cmp(cond, mode, a, b)` and `ap_cmps(cond, mode, a, s)`
with constants `AP_EQ` ... `AP_LE` and `AP_TSET`, `AP_TAND`, `AP_TOR`.

## funct3 = 3: tag operations and reductions

| funct7 | Name | Operands | Operation | C |
|---:|---|---|---|---|
| 0 | TAG_ALL | | tag = 1 for every row | `ap_tag_all()` |
| 1 | TAG_NONE | | tag = 0 | `ap_tag_none()` |
| 2 | TAG_NOT | | tag = ~tag | `ap_tag_not()` |
| 3 | TAG_RANGE | rs1 = lo, rs2 = hi | tag = (lo <= row < hi) | `ap_tag_range(lo, hi)` |
| 4 | TAG_SAVE | rs1[15:0] = field | bit `offset` of every row = tag | `ap_tag_save(f)` |
| 5 | TAG_LOAD | rs1[15:0] = field | tag = bit `offset` of every row | `ap_tag_load(f)` |
| 6 | COUNT | | rd = number of tagged rows | `ap_count()` |
| 7 | FIRST | | rd = lowest tagged row, or 0xFFFFFFFF | `ap_first()` |
| 8 | MAXU | rs1[15:0] = field a | see below | `ap_maxu(a)` |
| 9 | MINU | rs1[15:0] = field a | | `ap_minu(a)` |
| 10 | MAX | rs1[15:0] = field a | signed | `ap_max(a)` |
| 11 | MIN | rs1[15:0] = field a | signed | `ap_min(a)` |

TAG_SAVE and TAG_LOAD use only the offset of the field, so a 1-bit field is
the natural argument. TAG_RANGE compares 32-bit unsigned values.

**MAXU / MINU / MAX / MIN** search the tagged rows for the largest (smallest)
value of a, keep only the rows holding it tagged, and return the value in rd
(sign-extended for MAX and MIN). They visit the bits of a from the most
significant down: a candidate row stays if its bit is the preferred value (1
for a maximum, 0 for a minimum; inverted at the sign bit of signed searches),
unless no candidate has that value. If no row is tagged, rd = 0 and the tag
stays empty.

## funct3 = 4: element access

`rs1 = {row, field}`. Ignores the tag.

| funct7 | Name | Operation | C |
|---:|---|---|---|
| 0 | RD | rd = field of the row (zero-extended) | `ap_read(f, row)` |
| 1 | WR | field of the row = rs2 (low L bits) | `ap_write(f, row, v)` |

Reading a row beyond ROWS returns 0; writing one does nothing.

## funct3 = 5: transpose buffer

The array is stored in bit planes, so moving one word at a time costs L
cycles per word. The transpose buffer moves 32 words with one L-cycle
instruction: XPUT writes word k of the buffer into row `base + k`, XGET reads
it back. `base` is `row` rounded down to a multiple of 32.

| funct7 | Name | Operands | Operation | C |
|---:|---|---|---|---|
| 0 | XBUF_WR | rs1[4:0] = index, rs2 = value | xbuf[index] = rs2 | `ap_xwrite(i, v)` |
| 1 | XBUF_RD | rs1[4:0] = index | rd = xbuf[index] | `ap_xread(i)` |
| 2 | XPUT | rs1 = {row, field}, rs2 = n | rows base .. base+n-1: field = xbuf[0 .. n-1] | `ap_xput(f, row, n)` |
| 3 | XGET | rs1 = {row, field}, rs2 = n | xbuf[0 .. n-1] = field of rows base .. base+n-1, rest of xbuf = 0 | `ap_xget(f, row, n)` |

n = 0 or n >= 32 means 32. XPUT and XGET ignore the tag. `ap_put_array()` and
`ap_get_array()` in `aplib.c` wrap them for whole arrays.

## funct3 = 6: DMA

Moves arrays of 32-bit words between external memory (AXI) and a field. Word
i (at `address + 4*i`) belongs to row `base + i`; `base` is `row` rounded
down to a multiple of 32. Loads keep the low L bits of each word, stores
write L bits zero-extended to 32. The tag is ignored.

| funct7 | Name | Operands | Operation | C |
|---:|---|---|---|---|
| 0 | DMA_CFG | rs1 = address, rs2 = count | set the transfer address and element count | `ap_dma_config(addr, n)` |
| 1 | DMA_LOAD | rs1 = {row, field} | memory -> field; rd = status | `ap_dma_load(f, row)` |
| 2 | DMA_STORE | rs1 = {row, field} | field -> memory; rd = status | `ap_dma_store(f, row)` |

Status is 0 on success and 1 if the memory answered with an error. The
address is a CPU address in the external window (0x8000_0000 and up) and must
be word aligned. Address and count stay set, so several fields can be moved
with one DMA_CFG if they come from the same array. DMA_CFG writes 0 to rd.

## funct3 = 7: system

| funct7 | Name | Operands | Operation | C |
|---:|---|---|---|---|
| 0 | INFO | rs1 = selector | rd = ROWS (0), COLS (1), version (2), else 0 | `ap_rows()`, `ap_cols()`, `ap_version()` |

The version reads 0x00020000 (2.0.0).
