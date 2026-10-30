// The K largest values and their rows.
//
// MAXU narrows the tag to the rows holding the maximum in one pass over the
// field's bits, MSB first; FIRST picks one of them. A candidate mask kept in a
// 1-bit field removes the picked row before the next round.
#include "runtime.h"
#include "ap.h"

#define MAX_N 1024
#define K     8

static uint32_t x[MAX_N];
static uint8_t  used[MAX_N];

int main(void)
{
    uint32_t n = ap_rows();
    if (n > MAX_N) n = MAX_N;

    const ap_field_t fx = AP_FIELD(0, 20);
    const ap_field_t fc = AP_FIELD(20, 1);     // still a candidate

    uint32_t seed = 3;
    for (uint32_t i = 0; i < n; i++) x[i] = rand32(&seed) & 0xfffff;
    ap_tag_all();
    ap_put_array(fx, 0, x, n);

    // CPU: K passes, each finds the largest unused value (ties: lowest row)
    uint32_t cpu_row[K], cpu_val[K];
    uint32_t t0 = rdcycle();
    for (uint32_t j = 0; j < K; j++) {
        uint32_t best = 0, at = 0;
        int have = 0;
        for (uint32_t i = 0; i < n; i++)
            if (!used[i] && (!have || x[i] > best)) { best = x[i]; at = i; have = 1; }
        used[at] = 1;
        cpu_row[j] = at;
        cpu_val[j] = best;
    }
    uint32_t t_cpu = rdcycle() - t0;

    // AP
    uint32_t ap_row[K], ap_val[K];
    t0 = rdcycle();
    ap_tag_range(0, n);
    ap_tag_save(fc);
    for (uint32_t j = 0; j < K; j++) {
        ap_tag_load(fc);
        ap_val[j] = ap_maxu(fx);
        ap_row[j] = ap_first();
        ap_write(fc, ap_row[j], 0);
    }
    uint32_t t_ap = rdcycle() - t0;

    uint32_t errors = 0;
    printf_("top %u of %u 20-bit values\n", K, n);
    for (uint32_t j = 0; j < K; j++) {
        if (ap_val[j] != cpu_val[j] || ap_row[j] != cpu_row[j]) errors++;
        printf_("  #%u  row %4u  value %7u\n", j + 1, ap_row[j], ap_val[j]);
    }
    printf_("  CPU                 %7u cycles\n", t_cpu);
    printf_("  AP                  %7u cycles\n", t_ap);
    printf_("  mismatches          %7u\n", errors);
    return (int)errors;
}
