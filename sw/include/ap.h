// C interface to the AP instruction set (RISC-V custom-1 opcode).
//
// See docs/isa.md for the semantics. All functions compile to a single
// instruction; the CPU waits until the AP unit has finished it.
//
// A field names the same bit range in every row of the array:
//     ap_field_t f = AP_FIELD(offset, length);   // length 1..32
// Arithmetic and logic operations write only the rows whose tag bit is set;
// compares compute the tag.
#ifndef AP_H
#define AP_H

#include <stdint.h>

typedef uint32_t ap_field_t;

#define AP_FIELD(off, len)  ((ap_field_t)((((uint32_t)(len) - 1u) << 11) | ((uint32_t)(off) & 0x7ffu)))
#define AP_FIELD_OFF(f)     ((f) & 0x7ffu)
#define AP_FIELD_LEN(f)     ((((f) >> 11) & 0x1fu) + 1u)

// rs1 operand that carries two 16-bit halves
#define AP_PAIR(hi, lo)     ((((uint32_t)(hi)) << 16) | ((uint32_t)(lo) & 0xffffu))

// ------------------------------------------------------------ encodings
#define AP_GRP_VV    0
#define AP_GRP_VS    1
#define AP_GRP_CMP   2
#define AP_GRP_TAG   3
#define AP_GRP_ELEM  4
#define AP_GRP_XBUF  5
#define AP_GRP_DMA   6
#define AP_GRP_SYS   7

#define AP_ALU_ADD   0
#define AP_ALU_SUB   1
#define AP_ALU_AND   2
#define AP_ALU_OR    3
#define AP_ALU_XOR   4
#define AP_ALU_MOV   5
#define AP_ALU_NOT   6
#define AP_ALU_NEG   7
#define AP_ALU_ABS   8
#define AP_ALU_SET   9
#define AP_ALU_RSUB  10

// compare conditions
#define AP_EQ   0
#define AP_NE   1
#define AP_LTU  2
#define AP_LT   3
#define AP_GEU  4
#define AP_GE   5
#define AP_GTU  6
#define AP_GT   7
#define AP_LEU  8
#define AP_LE   9

// how a compare combines with the current tag
#define AP_TSET 0      // tag = cond
#define AP_TAND 1      // tag = tag & cond
#define AP_TOR  2      // tag = tag | cond

#define AP_TAG_ALL    0
#define AP_TAG_NONE   1
#define AP_TAG_NOT    2
#define AP_TAG_RANGE  3
#define AP_TAG_SAVE   4
#define AP_TAG_LOAD   5
#define AP_TAG_COUNT  6
#define AP_TAG_FIRST  7
#define AP_TAG_MAXU   8
#define AP_TAG_MINU   9
#define AP_TAG_MAX    10
#define AP_TAG_MIN    11

#define AP_INFO_ROWS     0
#define AP_INFO_COLS     1
#define AP_INFO_VERSION  2

#define AP_NONE  0xffffffffu   // ap_first() when no row is tagged

// Raw instruction forms: result in rd, or rd = x0.
#define AP_INSN(f3, f7, rs1, rs2) ({                                        \
        uint32_t ap_rd_;                                                    \
        __asm__ volatile (".insn r 0x2b, %3, %4, %0, %1, %2"                \
                          : "=r"(ap_rd_)                                    \
                          : "r"((uint32_t)(rs1)), "r"((uint32_t)(rs2)),     \
                            "i"(f3), "i"(f7));                              \
        ap_rd_; })

#define AP_INSN_NR(f3, f7, rs1, rs2)                                        \
        __asm__ volatile (".insn r 0x2b, %2, %3, x0, %0, %1"                \
                          :                                                 \
                          : "r"((uint32_t)(rs1)), "r"((uint32_t)(rs2)),     \
                            "i"(f3), "i"(f7))

// ------------------------------------------- field (vector) operations
// d = op(a, b) on every tagged row
static inline void ap_add (ap_field_t d, ap_field_t a, ap_field_t b) { AP_INSN_NR(AP_GRP_VV, AP_ALU_ADD,  AP_PAIR(a, d), b); }
static inline void ap_sub (ap_field_t d, ap_field_t a, ap_field_t b) { AP_INSN_NR(AP_GRP_VV, AP_ALU_SUB,  AP_PAIR(a, d), b); }
static inline void ap_rsub(ap_field_t d, ap_field_t a, ap_field_t b) { AP_INSN_NR(AP_GRP_VV, AP_ALU_RSUB, AP_PAIR(a, d), b); }
static inline void ap_and (ap_field_t d, ap_field_t a, ap_field_t b) { AP_INSN_NR(AP_GRP_VV, AP_ALU_AND,  AP_PAIR(a, d), b); }
static inline void ap_or  (ap_field_t d, ap_field_t a, ap_field_t b) { AP_INSN_NR(AP_GRP_VV, AP_ALU_OR,   AP_PAIR(a, d), b); }
static inline void ap_xor (ap_field_t d, ap_field_t a, ap_field_t b) { AP_INSN_NR(AP_GRP_VV, AP_ALU_XOR,  AP_PAIR(a, d), b); }
static inline void ap_copy(ap_field_t d, ap_field_t b)               { AP_INSN_NR(AP_GRP_VV, AP_ALU_SET,  AP_PAIR(0, d), b); }

// d = op(a)
static inline void ap_mov(ap_field_t d, ap_field_t a) { AP_INSN_NR(AP_GRP_VV, AP_ALU_MOV, AP_PAIR(a, d), 0); }
static inline void ap_not(ap_field_t d, ap_field_t a) { AP_INSN_NR(AP_GRP_VV, AP_ALU_NOT, AP_PAIR(a, d), 0); }
static inline void ap_neg(ap_field_t d, ap_field_t a) { AP_INSN_NR(AP_GRP_VV, AP_ALU_NEG, AP_PAIR(a, d), 0); }
static inline void ap_abs(ap_field_t d, ap_field_t a) { AP_INSN_NR(AP_GRP_VV, AP_ALU_ABS, AP_PAIR(a, d), 0); }

// d = op(a, s) with the scalar s broadcast to every tagged row
static inline void ap_adds (ap_field_t d, ap_field_t a, uint32_t s) { AP_INSN_NR(AP_GRP_VS, AP_ALU_ADD,  AP_PAIR(a, d), s); }
static inline void ap_subs (ap_field_t d, ap_field_t a, uint32_t s) { AP_INSN_NR(AP_GRP_VS, AP_ALU_SUB,  AP_PAIR(a, d), s); }
static inline void ap_rsubs(ap_field_t d, ap_field_t a, uint32_t s) { AP_INSN_NR(AP_GRP_VS, AP_ALU_RSUB, AP_PAIR(a, d), s); }
static inline void ap_ands (ap_field_t d, ap_field_t a, uint32_t s) { AP_INSN_NR(AP_GRP_VS, AP_ALU_AND,  AP_PAIR(a, d), s); }
static inline void ap_ors  (ap_field_t d, ap_field_t a, uint32_t s) { AP_INSN_NR(AP_GRP_VS, AP_ALU_OR,   AP_PAIR(a, d), s); }
static inline void ap_xors (ap_field_t d, ap_field_t a, uint32_t s) { AP_INSN_NR(AP_GRP_VS, AP_ALU_XOR,  AP_PAIR(a, d), s); }
static inline void ap_sets (ap_field_t d, uint32_t s)               { AP_INSN_NR(AP_GRP_VS, AP_ALU_SET,  AP_PAIR(0, d), s); }

// ----------------------------------------------------------- compares
// cond is AP_EQ ... AP_LE, mode is AP_TSET / AP_TAND / AP_TOR; both must be
// compile-time constants.
#define ap_cmp(cond, mode, a, b)  AP_INSN_NR(AP_GRP_CMP, ((mode) << 5) | (cond),            (a), (b))
#define ap_cmps(cond, mode, a, s) AP_INSN_NR(AP_GRP_CMP, ((mode) << 5) | (1 << 4) | (cond), (a), (s))

// ------------------------------------------------------- tag and search
static inline void     ap_tag_all(void)                      { AP_INSN_NR(AP_GRP_TAG, AP_TAG_ALL,   0, 0); }
static inline void     ap_tag_none(void)                     { AP_INSN_NR(AP_GRP_TAG, AP_TAG_NONE,  0, 0); }
static inline void     ap_tag_not(void)                      { AP_INSN_NR(AP_GRP_TAG, AP_TAG_NOT,   0, 0); }
static inline void     ap_tag_range(uint32_t lo, uint32_t hi) { AP_INSN_NR(AP_GRP_TAG, AP_TAG_RANGE, lo, hi); }
static inline void     ap_tag_save(ap_field_t bit)           { AP_INSN_NR(AP_GRP_TAG, AP_TAG_SAVE,  bit, 0); }
static inline void     ap_tag_load(ap_field_t bit)           { AP_INSN_NR(AP_GRP_TAG, AP_TAG_LOAD,  bit, 0); }
static inline uint32_t ap_count(void)                        { return AP_INSN(AP_GRP_TAG, AP_TAG_COUNT, 0, 0); }
static inline uint32_t ap_first(void)                        { return AP_INSN(AP_GRP_TAG, AP_TAG_FIRST, 0, 0); }

// Keep only the tagged rows holding the largest / smallest value of a and
// return that value (0 if no row is tagged).
static inline uint32_t ap_maxu(ap_field_t a) { return AP_INSN(AP_GRP_TAG, AP_TAG_MAXU, a, 0); }
static inline uint32_t ap_minu(ap_field_t a) { return AP_INSN(AP_GRP_TAG, AP_TAG_MINU, a, 0); }
static inline int32_t  ap_max (ap_field_t a) { return (int32_t)AP_INSN(AP_GRP_TAG, AP_TAG_MAX, a, 0); }
static inline int32_t  ap_min (ap_field_t a) { return (int32_t)AP_INSN(AP_GRP_TAG, AP_TAG_MIN, a, 0); }

// ------------------------------------------------------ element access
static inline uint32_t ap_read(ap_field_t f, uint32_t row)              { return AP_INSN(AP_GRP_ELEM, 0, AP_PAIR(row, f), 0); }
static inline void     ap_write(ap_field_t f, uint32_t row, uint32_t v) { AP_INSN_NR(AP_GRP_ELEM, 1, AP_PAIR(row, f), v); }

// ---------------------------------------------------- transpose buffer
// 32 words that move to/from 32 consecutive rows (row is rounded down to a
// multiple of 32) in about one cycle per field bit.
static inline void     ap_xwrite(uint32_t i, uint32_t v) { AP_INSN_NR(AP_GRP_XBUF, 0, i, v); }
static inline uint32_t ap_xread(uint32_t i)              { return AP_INSN(AP_GRP_XBUF, 1, i, 0); }
static inline void     ap_xput(ap_field_t f, uint32_t row, uint32_t n) { AP_INSN_NR(AP_GRP_XBUF, 2, AP_PAIR(row, f), n); }
static inline void     ap_xget(ap_field_t f, uint32_t row, uint32_t n) { AP_INSN_NR(AP_GRP_XBUF, 3, AP_PAIR(row, f), n); }

// ----------------------------------------------------------------- DMA
// Element i is the 32-bit word at addr + 4*i, row base + i (base a multiple
// of 32). Returns 0 on success, 1 on a bus error.
static inline void ap_dma_config(uint32_t addr, uint32_t count)
{
    __asm__ volatile (".insn r 0x2b, 6, 0, x0, %0, %1" :: "r"(addr), "r"(count) : "memory");
}

static inline uint32_t ap_dma_load(ap_field_t f, uint32_t base)
{
    uint32_t st;
    __asm__ volatile (".insn r 0x2b, 6, 1, %0, %1, x0" : "=r"(st) : "r"(AP_PAIR(base, f)) : "memory");
    return st;
}

static inline uint32_t ap_dma_store(ap_field_t f, uint32_t base)
{
    uint32_t st;
    __asm__ volatile (".insn r 0x2b, 6, 2, %0, %1, x0" : "=r"(st) : "r"(AP_PAIR(base, f)) : "memory");
    return st;
}

// --------------------------------------------------------------- info
static inline uint32_t ap_rows(void)    { return AP_INSN(AP_GRP_SYS, 0, AP_INFO_ROWS, 0); }
static inline uint32_t ap_cols(void)    { return AP_INSN(AP_GRP_SYS, 0, AP_INFO_COLS, 0); }
static inline uint32_t ap_version(void) { return AP_INSN(AP_GRP_SYS, 0, AP_INFO_VERSION, 0); }

// ------------------------------------------------- helpers (aplib.c)
// Move arrays between CPU memory and a field, 32 rows at a time through
// the transpose buffer. Rows first .. first+n-1, first a multiple of 32.
void ap_put_array(ap_field_t f, uint32_t first, const uint32_t *src, uint32_t n);
void ap_get_array(ap_field_t f, uint32_t first, uint32_t *dst, uint32_t n);

#endif
