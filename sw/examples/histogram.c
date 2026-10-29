// 16-bin histogram of 8-bit samples.
//
// The bin of a sample is its top four bits. Those bits are a field of their
// own (offset + 4, length 4), so each bin costs one 4-bit compare and one
// COUNT, independent of the number of samples. The rows in use are tagged
// once and the mask saved, so each bin restores it with a one-step TAG_LOAD.
#include "runtime.h"
#include "ap.h"

#define MAX_N 1024

static uint32_t x[MAX_N];

int main(void)
{
    uint32_t n = ap_rows();
    if (n > MAX_N) n = MAX_N;

    const ap_field_t fx   = AP_FIELD(0, 8);
    const ap_field_t fbin = AP_FIELD(4, 4);    // x >> 4
    const ap_field_t fvalid = AP_FIELD(8, 1);  // rows in use

    // a smooth, uneven distribution
    uint32_t seed = 99;
    for (uint32_t i = 0; i < n; i++) {
        uint32_t r = rand32(&seed);
        x[i] = ((r & 0xff) + ((r >> 8) & 0xff) + ((r >> 16) & 0x7f)) / 3;
    }
    ap_tag_all();
    ap_put_array(fx, 0, x, n);

    uint32_t h_cpu[16] = {0}, h_ap[16];

    uint32_t t0 = rdcycle();
    for (uint32_t i = 0; i < n; i++) h_cpu[x[i] >> 4]++;
    uint32_t t_cpu = rdcycle() - t0;

    t0 = rdcycle();
    ap_tag_range(0, n);
    ap_tag_save(fvalid);
    for (uint32_t b = 0; b < 16; b++) {
        ap_tag_load(fvalid);
        ap_cmps(AP_EQ, AP_TAND, fbin, b);
        h_ap[b] = ap_count();
    }
    uint32_t t_ap = rdcycle() - t0;

    uint32_t errors = 0;
    printf_("histogram of %u 8-bit samples\n", n);
    for (uint32_t b = 0; b < 16; b++) {
        if (h_ap[b] != h_cpu[b]) errors++;
        printf_("  %3u..%3u %4u ", b * 16, b * 16 + 15, h_ap[b]);
        for (uint32_t s = 0; s < h_ap[b] * 64 / n; s++) putchar_('#');
        putchar_('\n');
    }
    printf_("  CPU loop            %7u cycles\n", t_cpu);
    printf_("  AP                  %7u cycles\n", t_ap);
    printf_("  mismatches          %7u\n", errors);
    return (int)errors;
}
