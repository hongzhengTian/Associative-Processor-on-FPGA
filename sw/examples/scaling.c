// Scaling benchmark: AP time of five workloads (bench.h) for n = 32, 64, ...
// up to the number of rows.
//
// Prints one line per measurement:
//     BENCH <workload> <n> <cycles> <result>
// The Zynq host program reads these lines and runs the same workloads on the
// ARM core for comparison. The data is loaded into the array once and is not
// part of the timing; each measurement starts by tagging rows 0 .. n-1.
// Every AP result is checked against the C version of bench.h; the exit code
// is the number of mismatches.
#include "runtime.h"
#include "ap.h"
#include "bench.h"

static uint32_t a[BENCH_MAX_N], b[BENCH_MAX_N], c[BENCH_MAX_N];
static uint32_t key[BENCH_MAX_N], val[BENCH_MAX_N];
static uint32_t x8[BENCH_MAX_N], x20[BENCH_MAX_N];

static uint32_t errors;

static void report(const char *name, uint32_t n, uint32_t cycles, uint32_t result, uint32_t expected)
{
    printf_("BENCH %s %u %u %u\n", name, n, cycles, result);
    if (result != expected) {
        printf_("  mismatch: expected %u\n", expected);
        errors++;
    }
}

int main(void)
{
    uint32_t rows = ap_rows();
    uint32_t max_n = rows < BENCH_MAX_N ? rows : BENCH_MAX_N;

    bench_fill(a,   max_n, BENCH_SEED_A,   16);
    bench_fill(b,   max_n, BENCH_SEED_B,   16);
    bench_fill(key, max_n, BENCH_SEED_KEY, 12);
    bench_fill(val, max_n, BENCH_SEED_VAL, 16);
    bench_fill(x8,  max_n, BENCH_SEED_X8,   8);
    bench_fill(x20, max_n, BENCH_SEED_X20, 20);

    printf_("scaling benchmark, %u rows x %u columns\n", rows, ap_cols());

    // ------------------------------------------------------------ add
    const ap_field_t fa = AP_FIELD(0, 16), fb = AP_FIELD(16, 16), fc = AP_FIELD(32, 17);
    ap_tag_all();
    ap_put_array(fa, 0, a, max_n);
    ap_put_array(fb, 0, b, max_n);
    for (uint32_t n = 32; n <= max_n; n *= 2) {
        uint32_t t0 = rdcycle();
        ap_tag_range(0, n);
        ap_add(fc, fa, fb);
        uint32_t t = rdcycle() - t0;
        ap_get_array(fc, 0, c, n);
        uint32_t result = bench_sum(c, n);
        bench_add(a, b, c, n);
        report("add", n, t, result, bench_sum(c, n));
    }

    // ---------------------------------------------------------- range
    const ap_field_t fkey = AP_FIELD(0, 12), fval = AP_FIELD(12, 16);
    ap_tag_all();
    ap_put_array(fkey, 0, key, max_n);
    ap_put_array(fval, 0, val, max_n);
    for (uint32_t n = 32; n <= max_n; n *= 2) {
        uint32_t t0 = rdcycle();
        ap_tag_range(0, n);
        ap_cmps(AP_GEU, AP_TAND, fkey, BENCH_KEY_LO);
        ap_cmps(AP_LTU, AP_TAND, fkey, BENCH_KEY_HI);
        ap_cmps(AP_GTU, AP_TAND, fval, BENCH_VAL_MIN);
        uint32_t result = ap_count();
        uint32_t t = rdcycle() - t0;
        report("range", n, t, result, bench_range(key, val, n));
    }

    // ---------------------------------------------------------- argmax
    const ap_field_t fx = AP_FIELD(0, 20), fcand = AP_FIELD(20, 1);
    ap_tag_all();
    ap_put_array(fx, 0, x20, max_n);
    for (uint32_t n = 32; n <= max_n; n *= 2) {
        uint32_t t0 = rdcycle();
        ap_tag_range(0, n);
        uint32_t v = ap_maxu(fx);
        uint32_t r = ap_first();
        uint32_t t = rdcycle() - t0;
        report("argmax", n, t, v | (r << 20), bench_argmax(x20, n));
    }

    // ------------------------------------------------------------ top8
    for (uint32_t n = 32; n <= max_n; n *= 2) {
        uint32_t idx[8];
        uint32_t t0 = rdcycle();
        ap_tag_range(0, n);
        ap_tag_save(fcand);
        for (uint32_t j = 0; j < 8; j++) {
            ap_tag_load(fcand);
            ap_maxu(fx);
            idx[j] = ap_first();
            ap_write(fcand, idx[j], 0);
        }
        uint32_t t = rdcycle() - t0;
        uint32_t result = 0;
        for (uint32_t j = 0; j < 8; j++) result += (j + 1) * (idx[j] + 1);
        report("top8", n, t, result, bench_top8(x20, n));
    }

    // ---------------------------------------------------------- hist16
    const ap_field_t fx8 = AP_FIELD(0, 8), fbin = AP_FIELD(4, 4), fvalid = AP_FIELD(8, 1);
    ap_tag_all();
    ap_put_array(fx8, 0, x8, max_n);
    for (uint32_t n = 32; n <= max_n; n *= 2) {
        uint32_t bins[16];
        uint32_t t0 = rdcycle();
        ap_tag_range(0, n);
        ap_tag_save(fvalid);
        for (uint32_t bin = 0; bin < 16; bin++) {
            ap_tag_load(fvalid);
            ap_cmps(AP_EQ, AP_TAND, fbin, bin);
            bins[bin] = ap_count();
        }
        uint32_t t = rdcycle() - t0;
        uint32_t result = 0;
        for (uint32_t bin = 0; bin < 16; bin++) result += (bin + 1) * bins[bin];
        report("hist16", n, t, result, bench_hist16(x8, n));
    }

    printf_("%u mismatches\n", errors);
    return (int)errors;
}
