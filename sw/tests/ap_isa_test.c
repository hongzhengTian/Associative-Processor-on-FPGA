// System-level test of the AP instruction set: every instruction group runs
// through PCPI on the real SoC and its result is checked against the same
// computation done by the CPU. Exit code = number of failed checks.
#include "runtime.h"
#include "ap.h"

#define MAX_ROWS 1024

static uint32_t rows;
static uint32_t errors;
static uint32_t seed = 0x1234567u;

static uint32_t va[MAX_ROWS], vb[MAX_ROWS], vd[MAX_ROWS], got[MAX_ROWS];
static uint8_t  tag[MAX_ROWS];

static const ap_field_t FA = AP_FIELD(0, 16);
static const ap_field_t FB = AP_FIELD(16, 16);
static const ap_field_t FD = AP_FIELD(32, 16);
static const ap_field_t FT = AP_FIELD(48, 1);     // saved tag

static void check(int ok, const char *what, uint32_t row, uint32_t g, uint32_t e)
{
    if (ok) return;
    if (errors < 20)
        printf_("  FAIL %s row %u: got %08x expected %08x\n", what, row, g, e);
    errors++;
}

static uint32_t mask(uint32_t len) { return len >= 32 ? 0xffffffffu : (1u << len) - 1u; }

static int32_t sx(uint32_t v, uint32_t len)
{
    return (int32_t)(v << (32 - len)) >> (32 - len);
}

// Random 16-bit data with some extreme values mixed in.
static void fill(uint32_t *v)
{
    for (uint32_t r = 0; r < rows; r++) {
        uint32_t x = rand32(&seed);
        switch (x & 7) {
        case 0:  v[r] = 0; break;
        case 1:  v[r] = 0xffff; break;
        case 2:  v[r] = 0x8000; break;
        default: v[r] = (x >> 8) & 0xffff; break;
        }
    }
}

// Read the tag back through a saved bit field.
static void read_tag(uint8_t *t)
{
    ap_tag_save(FT);
    ap_get_array(FT, 0, got, rows);
    for (uint32_t r = 0; r < rows; r++) t[r] = (uint8_t)got[r];
}

static void check_tag(const char *what)
{
    uint8_t t[MAX_ROWS];
    read_tag(t);
    for (uint32_t r = 0; r < rows; r++) check(t[r] == tag[r], what, r, t[r], tag[r]);
}

static void check_field(ap_field_t f, const uint32_t *exp, const char *what)
{
    ap_get_array(f, 0, got, rows);
    for (uint32_t r = 0; r < rows; r++) check(got[r] == exp[r], what, r, got[r], exp[r]);
}

// Tag the rows with a < t (also exercises a compare) and mirror it.
static void tag_lt(uint32_t t)
{
    ap_cmps(AP_LTU, AP_TSET, FA, t);
    for (uint32_t r = 0; r < rows; r++) tag[r] = va[r] < t;
}

// ------------------------------------------------------------- arithmetic
static uint32_t ref_alu(int op, uint32_t a, uint32_t b)
{
    switch (op) {
    case AP_ALU_ADD:  return a + b;
    case AP_ALU_SUB:  return a - b;
    case AP_ALU_AND:  return a & b;
    case AP_ALU_OR:   return a | b;
    case AP_ALU_XOR:  return a ^ b;
    case AP_ALU_MOV:  return a;
    case AP_ALU_NOT:  return ~a;
    case AP_ALU_NEG:  return -a;
    case AP_ALU_ABS:  return (a & 0x8000) ? -a : a;
    case AP_ALU_SET:  return b;
    default:          return b - a;     // RSUB
    }
}

static void run_vv(int op)
{
    switch (op) {
    case AP_ALU_ADD:  ap_add(FD, FA, FB);  break;
    case AP_ALU_SUB:  ap_sub(FD, FA, FB);  break;
    case AP_ALU_AND:  ap_and(FD, FA, FB);  break;
    case AP_ALU_OR:   ap_or(FD, FA, FB);   break;
    case AP_ALU_XOR:  ap_xor(FD, FA, FB);  break;
    case AP_ALU_MOV:  ap_mov(FD, FA);      break;
    case AP_ALU_NOT:  ap_not(FD, FA);      break;
    case AP_ALU_NEG:  ap_neg(FD, FA);      break;
    case AP_ALU_ABS:  ap_abs(FD, FA);      break;
    case AP_ALU_SET:  ap_copy(FD, FB);     break;
    default:          ap_rsub(FD, FA, FB); break;
    }
}

static void run_vs(int op, uint32_t s)
{
    switch (op) {
    case AP_ALU_ADD:  ap_adds(FD, FA, s);  break;
    case AP_ALU_SUB:  ap_subs(FD, FA, s);  break;
    case AP_ALU_AND:  ap_ands(FD, FA, s);  break;
    case AP_ALU_OR:   ap_ors(FD, FA, s);   break;
    case AP_ALU_XOR:  ap_xors(FD, FA, s);  break;
    case AP_ALU_SET:  ap_sets(FD, s);      break;
    default:          ap_rsubs(FD, FA, s); break;
    }
}

static void test_alu(void)
{
    static const int vs_ops[] = { AP_ALU_ADD, AP_ALU_SUB, AP_ALU_AND, AP_ALU_OR,
                                  AP_ALU_XOR, AP_ALU_SET, AP_ALU_RSUB };
    puts_("arithmetic and logic\n");
    for (int op = 0; op <= AP_ALU_RSUB; op++) {
        fill(va); fill(vb); fill(vd);
        ap_tag_all();
        ap_put_array(FA, 0, va, rows);
        ap_put_array(FB, 0, vb, rows);
        ap_put_array(FD, 0, vd, rows);
        tag_lt(0x9000);
        run_vv(op);
        for (uint32_t r = 0; r < rows; r++)
            if (tag[r]) vd[r] = ref_alu(op, va[r], vb[r]) & 0xffff;
        check_field(FD, vd, "vector op");
    }
    for (unsigned i = 0; i < sizeof vs_ops / sizeof vs_ops[0]; i++) {
        int op = vs_ops[i];
        uint32_t s = rand32(&seed) & 0xffff;
        fill(va); fill(vd);
        ap_tag_all();
        ap_put_array(FA, 0, va, rows);
        ap_put_array(FD, 0, vd, rows);
        tag_lt(0xc000);
        run_vs(op, s);
        for (uint32_t r = 0; r < rows; r++)
            if (tag[r]) vd[r] = ref_alu(op, va[r], s) & 0xffff;
        check_field(FD, vd, "scalar op");
    }
}

// --------------------------------------------------------------- compares
static int ref_cmp(int cond, uint32_t a, uint32_t b)
{
    int32_t sa = sx(a, 16), sb = sx(b, 16);
    switch (cond) {
    case AP_EQ:  return a == b;
    case AP_NE:  return a != b;
    case AP_LTU: return a <  b;
    case AP_LT:  return sa <  sb;
    case AP_GEU: return a >= b;
    case AP_GE:  return sa >= sb;
    case AP_GTU: return a >  b;
    case AP_GT:  return sa >  sb;
    case AP_LEU: return a <= b;
    default:     return sa <= sb;
    }
}

#define CMP_CASE(c)                                                         \
    case c:                                                                 \
        if (scalar) ap_cmps(c, AP_TSET, FA, s); else ap_cmp(c, AP_TSET, FA, FB); \
        break;

static void run_cmp(int cond, int scalar, uint32_t s)
{
    switch (cond) {
    CMP_CASE(AP_EQ) CMP_CASE(AP_NE) CMP_CASE(AP_LTU) CMP_CASE(AP_LT)
    CMP_CASE(AP_GEU) CMP_CASE(AP_GE) CMP_CASE(AP_GTU) CMP_CASE(AP_GT)
    CMP_CASE(AP_LEU) CMP_CASE(AP_LE)
    }
}

static void test_cmp(void)
{
    puts_("compares\n");
    fill(va); fill(vb);
    for (uint32_t r = 0; r < rows; r += 7) vb[r] = va[r];          // some equal pairs
    ap_put_array(FA, 0, va, rows);
    ap_put_array(FB, 0, vb, rows);
    for (int scalar = 0; scalar < 2; scalar++) {
        for (int cond = AP_EQ; cond <= AP_LE; cond++) {
            uint32_t s = scalar ? va[rand32(&seed) % rows] : 0;
            run_cmp(cond, scalar, s);
            for (uint32_t r = 0; r < rows; r++)
                tag[r] = (uint8_t)ref_cmp(cond, va[r], scalar ? s : vb[r]);
            check_tag(scalar ? "compare with scalar" : "compare fields");
        }
    }
    // combining modes: (a >= 0x4000) and (a < 0xc000), or a == 0
    ap_cmps(AP_GEU, AP_TSET, FA, 0x4000);
    ap_cmps(AP_LTU, AP_TAND, FA, 0xc000);
    ap_cmps(AP_EQ,  AP_TOR,  FA, 0);
    for (uint32_t r = 0; r < rows; r++)
        tag[r] = (va[r] >= 0x4000 && va[r] < 0xc000) || va[r] == 0;
    check_tag("compare modes");
}

// ------------------------------------------------- tag ops and reductions
static void test_tag(void)
{
    puts_("tag operations and reductions\n");
    ap_tag_range(3, rows - 5);
    for (uint32_t r = 0; r < rows; r++) tag[r] = r >= 3 && r < rows - 5;
    check_tag("tag range");
    ap_tag_not();
    for (uint32_t r = 0; r < rows; r++) tag[r] = !tag[r];
    check_tag("tag not");

    uint32_t n = ap_count(), f = ap_first(), en = 0, ef = AP_NONE;
    for (uint32_t r = 0; r < rows; r++)
        if (tag[r]) { en++; if (ef == AP_NONE) ef = r; }
    check(n == en, "count", 0, n, en);
    check(f == ef, "first", 0, f, ef);
    ap_tag_none();
    check(ap_count() == 0, "count none", 0, ap_count(), 0);
    check(ap_first() == AP_NONE, "first none", 0, ap_first(), AP_NONE);
    ap_tag_all();
    check(ap_count() == rows, "count all", 0, ap_count(), rows);

    // save, change, restore
    ap_tag_range(10, 20);
    ap_tag_save(AP_FIELD(60, 1));
    ap_tag_all();
    ap_tag_load(AP_FIELD(60, 1));
    for (uint32_t r = 0; r < rows; r++) tag[r] = r >= 10 && r < 20;
    check_tag("tag save/load");

    // max / min among the rows with a < 0xa000
    fill(va);
    va[rows / 2] = 0x7fff;
    ap_tag_all();
    ap_put_array(FA, 0, va, rows);
    for (int kind = 0; kind < 4; kind++) {
        tag_lt(0xa000);
        uint32_t best = 0;
        int have = 0;
        for (uint32_t r = 0; r < rows; r++) {
            if (!tag[r]) continue;
            uint32_t v = va[r];
            int better;
            if (kind < 2) better = kind == 0 ? v > best : v < best;
            else          better = kind == 2 ? sx(v, 16) > sx(best, 16) : sx(v, 16) < sx(best, 16);
            if (!have || better) best = v;
            have = 1;
        }
        uint32_t g;
        switch (kind) {
        case 0:  g = ap_maxu(FA); break;
        case 1:  g = ap_minu(FA); break;
        case 2:  g = (uint32_t)ap_max(FA); break;
        default: g = (uint32_t)ap_min(FA); break;
        }
        uint32_t e = kind < 2 ? best : (uint32_t)sx(best, 16);
        check(g == e, "max/min value", (uint32_t)kind, g, e);
        for (uint32_t r = 0; r < rows; r++) tag[r] = tag[r] && va[r] == best;
        check_tag("max/min rows");
    }
}

// ---------------------------------------------- element and xbuf access
static void test_access(void)
{
    puts_("element and transpose buffer access\n");
    ap_tag_all();
    for (uint32_t i = 0; i < 40; i++) {
        uint32_t r = rand32(&seed) % rows, v = rand32(&seed);
        ap_field_t f = AP_FIELD(i % 32, 1 + (i % 32));
        ap_write(f, r, v);
        uint32_t g = ap_read(f, r);
        check(g == (v & mask(AP_FIELD_LEN(f))), "element", r, g, v & mask(AP_FIELD_LEN(f)));
    }
    for (uint32_t i = 0; i < 32; i++) ap_xwrite(i, i * 0x01010101u);
    for (uint32_t i = 0; i < 32; i++) check(ap_xread(i) == i * 0x01010101u, "xbuf", i, ap_xread(i), i * 0x01010101u);

    // partial segment: only the first 5 rows of segment 1 change
    ap_field_t f = AP_FIELD(0, 32);
    fill(vd);
    ap_put_array(f, 0, vd, rows);
    for (uint32_t i = 0; i < 32; i++) ap_xwrite(i, 0xabc00000u + i);
    ap_xput(f, 32, 5);
    for (uint32_t i = 0; i < 5; i++) vd[32 + i] = 0xabc00000u + i;
    check_field(f, vd, "partial xput");
}

// ------------------------------------------------------------------- DMA
static void test_dma(void)
{
    puts_("DMA\n");
    // a source array that crosses a 4 KiB boundary
    volatile uint32_t *src = (volatile uint32_t *)(EXT_MEM_BASE + 0x2000 - 40);
    volatile uint32_t *dst = (volatile uint32_t *)(EXT_MEM_BASE + 0x10000);
    uint32_t n = rows - 3;
    ap_field_t f = AP_FIELD(0, 24), g = AP_FIELD(32, 24);

    fill(va);
    for (uint32_t r = 0; r < n; r++) src[r] = va[r] | 0xff000000u;   // high bits dropped
    ap_tag_all();
    ap_sets(f, 0);
    ap_dma_config((uint32_t)src, n);
    check(ap_dma_load(f, 0) == 0, "dma load status", 0, 1, 0);
    ap_get_array(f, 0, got, rows);
    for (uint32_t r = 0; r < rows; r++) {
        uint32_t e = r < n ? (va[r] | 0xff000000u) & 0xffffff : 0;
        check(got[r] == e, "dma load", r, got[r], e);
    }

    ap_adds(g, f, 0x100000);
    ap_dma_config((uint32_t)dst, n);
    check(ap_dma_store(g, 0) == 0, "dma store status", 0, 1, 0);
    for (uint32_t r = 0; r < n; r++) {
        uint32_t e = ((va[r] | 0xff000000u) + 0x100000) & 0xffffff;
        check(dst[r] == e, "dma store", r, dst[r], e);
    }

    // beyond the end of the 4 MiB external memory: bus error status
    ap_dma_config(EXT_MEM_BASE + (4u << 20) - 8, 4);
    check(ap_dma_load(f, 0) == 1, "dma error status", 0, 0, 1);
}

int main(void)
{
    rows = ap_rows();
    printf_("AP ISA test: %u rows, %u columns\n", rows, ap_cols());
    if (rows > MAX_ROWS || ap_cols() < 64) {
        puts_("needs at most 1024 rows and at least 64 columns\n");
        return 1;
    }
    check(ap_version() == 0x00020000u, "version", 0, ap_version(), 0x00020000u);

    test_alu();
    test_cmp();
    test_tag();
    test_access();
    test_dma();

    printf_("%u errors\n", errors);
    return (int)errors;
}
