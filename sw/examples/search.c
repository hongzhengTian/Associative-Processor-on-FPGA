// Associative search in a table of (key, value) records, one record per row.
//
//   1. exact match: all rows with key == k, then fetch their values
//   2. range query: lo <= key < hi and value > v, counted
//
// A compare marks every matching row at once; the CPU then walks the matches
// with FIRST and clears each one from a saved copy of the tag.
#include "runtime.h"
#include "ap.h"

#define MAX_N 1024

static uint32_t keys[MAX_N], vals[MAX_N];

int main(void)
{
    uint32_t n = ap_rows();
    if (n > MAX_N) n = MAX_N;

    const ap_field_t fk = AP_FIELD(0, 12);    // key
    const ap_field_t fv = AP_FIELD(12, 16);   // value
    const ap_field_t fm = AP_FIELD(28, 1);    // match mask

    uint32_t seed = 7;
    for (uint32_t i = 0; i < n; i++) {
        keys[i] = rand32(&seed) % 300;        // plenty of duplicates
        vals[i] = rand32(&seed) & 0xffff;
    }
    ap_tag_all();
    ap_put_array(fk, 0, keys, n);
    ap_put_array(fv, 0, vals, n);
    ap_tag_range(0, n);

    uint32_t errors = 0;
    const uint32_t k = keys[n / 3];

    // ---- exact match, CPU
    uint32_t cpu_hits[16], cpu_cnt = 0;
    uint32_t t0 = rdcycle();
    for (uint32_t i = 0; i < n; i++)
        if (keys[i] == k) {
            if (cpu_cnt < 16) cpu_hits[cpu_cnt] = vals[i];
            cpu_cnt++;
        }
    uint32_t t_cpu1 = rdcycle() - t0;

    // ---- exact match, AP
    uint32_t ap_hits[16], ap_cnt;
    t0 = rdcycle();
    ap_tag_range(0, n);
    ap_cmps(AP_EQ, AP_TAND, fk, k);
    ap_cnt = ap_count();
    ap_tag_save(fm);
    for (uint32_t j = 0; j < ap_cnt && j < 16; j++) {
        uint32_t r = ap_first();
        ap_hits[j] = ap_read(fv, r);
        ap_write(fm, r, 0);
        ap_tag_load(fm);
    }
    uint32_t t_ap1 = rdcycle() - t0;

    if (ap_cnt != cpu_cnt) errors++;
    for (uint32_t j = 0; j < cpu_cnt && j < 16; j++)
        if (ap_hits[j] != cpu_hits[j]) errors++;

    // ---- range query, CPU
    const uint32_t lo = 100, hi = 200, v = 0x8000;
    t0 = rdcycle();
    uint32_t cpu_range = 0;
    for (uint32_t i = 0; i < n; i++)
        if (keys[i] >= lo && keys[i] < hi && vals[i] > v) cpu_range++;
    uint32_t t_cpu2 = rdcycle() - t0;

    // ---- range query, AP
    t0 = rdcycle();
    ap_tag_range(0, n);
    ap_cmps(AP_GEU, AP_TAND, fk, lo);
    ap_cmps(AP_LTU, AP_TAND, fk, hi);
    ap_cmps(AP_GTU, AP_TAND, fv, v);
    uint32_t ap_range = ap_count();
    uint32_t t_ap2 = rdcycle() - t0;

    if (ap_range != cpu_range) errors++;

    printf_("associative search, %u records\n", n);
    printf_("  key == %u: %u matches\n", k, ap_cnt);
    printf_("    CPU scan          %7u cycles\n", t_cpu1);
    printf_("    AP                %7u cycles (search and fetch)\n", t_ap1);
    printf_("  %u <= key < %u and value > %u: %u matches\n", lo, hi, v, ap_range);
    printf_("    CPU scan          %7u cycles\n", t_cpu2);
    printf_("    AP                %7u cycles\n", t_ap2);
    printf_("  mismatches          %7u\n", errors);
    return (int)errors;
}
