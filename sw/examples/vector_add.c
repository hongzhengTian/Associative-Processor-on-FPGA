// c[i] = a[i] + b[i] for one element per row.
//
// The CPU version loops over arrays in on-chip RAM. The AP version moves the
// arrays from external memory with DMA, adds all rows in one instruction and
// writes the result back. Cycle counts are printed for both.
#include "runtime.h"
#include "ap.h"

#define MAX_N 1024

static uint32_t a[MAX_N], b[MAX_N], c[MAX_N];

int main(void)
{
    uint32_t n = ap_rows();
    if (n > MAX_N) n = MAX_N;

    volatile uint32_t *ea = (volatile uint32_t *)(EXT_MEM_BASE + 0x00000);
    volatile uint32_t *eb = (volatile uint32_t *)(EXT_MEM_BASE + 0x10000);
    volatile uint32_t *ec = (volatile uint32_t *)(EXT_MEM_BASE + 0x20000);

    uint32_t seed = 42;
    for (uint32_t i = 0; i < n; i++) {
        a[i] = ea[i] = rand32(&seed) & 0xffff;
        b[i] = eb[i] = rand32(&seed) & 0xffff;
    }

    // CPU
    uint32_t t0 = rdcycle();
    for (uint32_t i = 0; i < n; i++) c[i] = a[i] + b[i];
    uint32_t t_cpu = rdcycle() - t0;

    // AP: 16-bit operands, 17-bit sum
    const ap_field_t fa = AP_FIELD(0, 16), fb = AP_FIELD(16, 16), fc = AP_FIELD(32, 17);
    ap_tag_all();

    uint32_t t1 = rdcycle();
    ap_dma_config((uint32_t)ea, n);
    ap_dma_load(fa, 0);
    ap_dma_config((uint32_t)eb, n);
    ap_dma_load(fb, 0);
    uint32_t t2 = rdcycle();
    ap_add(fc, fa, fb);
    uint32_t t3 = rdcycle();
    ap_dma_config((uint32_t)ec, n);
    ap_dma_store(fc, 0);
    uint32_t t4 = rdcycle();

    uint32_t errors = 0;
    for (uint32_t i = 0; i < n; i++)
        if (ec[i] != c[i]) errors++;

    printf_("vector add, %u elements\n", n);
    printf_("  CPU loop            %7u cycles\n", t_cpu);
    printf_("  AP add              %7u cycles\n", t3 - t2);
    printf_("  AP with DMA in/out  %7u cycles\n", (t2 - t1) + (t4 - t2));
    printf_("  mismatches          %7u\n", errors);
    return (int)errors;
}
