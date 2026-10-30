// Host program for the AP SoC on a Zynq-7000 (bare metal, Cortex-A9).
//
// Runs every program of ap_programs.h on the SoC in the programmable logic:
// loads it into the SoC's RAM, starts it, prints its console output and its
// exit code. After the scaling benchmark it runs the same workloads
// (bench.h) on the Cortex-A9 and prints both timings side by side.
//
// Build: a Vitis "standalone" application for ps7_cortexa9_0 containing this
// file, ap_regs.h, ap_programs.h and sw/include/bench.h (docs/zynq.md).
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "xil_io.h"
#include "xil_mmu.h"
#include "xil_cache.h"
#include "xtime_l.h"
#include "ap_regs.h"
#include "ap_programs.h"
#include "bench.h"

// External memory of the SoC: 4 MiB of DDR. The SoC reaches it through the
// HP0 port, which does not snoop the ARM caches, so the ARM maps it uncached.
#define EXT_SIZE     (4u << 20)
#define TIMEOUT_SEC  60u

static uint32_t ext_mem[EXT_SIZE / 4] __attribute__((aligned(1 << 20)));

static char     capture[32768];
static uint32_t capture_len;

static inline void ap_wr(uint32_t off, uint32_t v) { Xil_Out32(AP_BASE + off, v); }
static inline uint32_t ap_rd(uint32_t off)         { return Xil_In32(AP_BASE + off); }

// ARM global timer, and conversion of short intervals to ns.
static uint64_t ticks(void)
{
    XTime t;
    XTime_GetTime(&t);
    return t;
}

static uint64_t ticks_to_ns(uint64_t dt)
{
    return dt * 1000000000ull / COUNTS_PER_SECOND;
}

// SoC clock, measured against the ARM global timer over 100 ms.
static uint32_t measure_clock_hz(void)
{
    uint64_t t0 = ticks();
    uint32_t c0 = ap_rd(AP_CLK_COUNT);
    while (ticks() - t0 < COUNTS_PER_SECOND / 10) ;
    uint32_t c1 = ap_rd(AP_CLK_COUNT);
    uint64_t t1 = ticks();
    return (uint32_t)((uint64_t)(c1 - c0) * COUNTS_PER_SECOND / (t1 - t0));
}

static void drain_console(void)
{
    uint32_t c;
    while ((c = ap_rd(AP_CONSOLE)) & AP_CONSOLE_VALID) {
        putchar((int)(c & 0xff));
        if (capture_len < sizeof capture - 1) capture[capture_len++] = (char)c;
    }
}

// Loads and runs one program; returns its exit code (-1: trap or timeout).
static int run_program(const struct ap_program *p, uint32_t clk_hz)
{
    ap_wr(AP_CTRL, 0);
    for (uint32_t i = 0; i < p->nwords; i++) ap_wr(AP_RAM + 4 * i, p->words[i]);
    for (uint32_t i = 0; i < p->nwords; i++)
        if (ap_rd(AP_RAM + 4 * i) != p->words[i]) {
            printf("program RAM readback mismatch at word %u\n", (unsigned)i);
            return -1;
        }

    memset(ext_mem, 0, EXT_SIZE);
    ap_wr(AP_EXT_BASE, (uint32_t)(uintptr_t)ext_mem);
    ap_wr(AP_EXT_SIZE, EXT_SIZE);

    printf("\n=== %s ===\n", p->name);
    capture_len = 0;
    uint64_t t0 = ticks();
    ap_wr(AP_CTRL, 1);

    uint32_t status;
    for (;;) {
        drain_console();
        status = ap_rd(AP_STATUS);
        if (status & (AP_STATUS_DONE | AP_STATUS_TRAP)) break;
        if (ticks() - t0 > (uint64_t)TIMEOUT_SEC * COUNTS_PER_SECOND) break;
    }
    drain_console();
    capture[capture_len] = '\0';

    // Runs here last well below 2^32 cycles; the high word is not needed.
    uint32_t cycles = ap_rd(AP_RUN_CYCLES_LO);
    int code = (int)ap_rd(AP_EXIT);
    ap_wr(AP_CTRL, 0);

    if (status & AP_STATUS_TRAP) {
        printf("[%s] CPU trap after %u cycles\n", p->name, (unsigned)cycles);
        return -1;
    }
    if (!(status & AP_STATUS_DONE)) {
        printf("[%s] timeout after %u s\n", p->name, (unsigned)TIMEOUT_SEC);
        return -1;
    }
    printf("[%s] exit code %d, %u cycles (%u us)\n", p->name, code, (unsigned)cycles,
           (unsigned)((uint64_t)cycles * 1000000ull / clk_hz));
    return code;
}

// ----------------------------------------------------------- scaling benchmark
static uint32_t a[BENCH_MAX_N], b[BENCH_MAX_N], c[BENCH_MAX_N];
static uint32_t key[BENCH_MAX_N], val[BENCH_MAX_N];
static uint32_t x8[BENCH_MAX_N], x20[BENCH_MAX_N];

// One run of a workload on the ARM; the barrier keeps the compiler from
// merging or hoisting repeated runs.
static uint32_t arm_run(const char *name, uint32_t n)
{
    __asm__ volatile ("" ::: "memory");
    if (!strcmp(name, "add"))    { bench_add(a, b, c, n); return bench_sum(c, n); }
    if (!strcmp(name, "range"))  return bench_range(key, val, n);
    if (!strcmp(name, "argmax")) return bench_argmax(x20, n);
    if (!strcmp(name, "top8"))   return bench_top8(x20, n);
    if (!strcmp(name, "hist16")) return bench_hist16(x8, n);
    return 0xffffffffu;
}

// Time of one run in ns: the add workload is timed without its checksum,
// as on the AP side.
static uint64_t arm_time_ns(const char *name, uint32_t n)
{
    const uint32_t reps = 200;
    arm_run(name, n);                                   // warm caches
    uint64_t t0 = ticks();
    for (uint32_t r = 0; r < reps; r++) {
        __asm__ volatile ("" ::: "memory");
        if (!strcmp(name, "add")) bench_add(a, b, c, n);
        else (void)arm_run(name, n);
    }
    return ticks_to_ns(ticks() - t0) / reps;
}

static void print_us(uint64_t ns)
{
    printf("%7u.%02u", (unsigned)(ns / 1000), (unsigned)(ns % 1000 / 10));
}

static int compare_scaling(uint32_t clk_hz)
{
    int mismatches = 0;

    bench_fill(a,   BENCH_MAX_N, BENCH_SEED_A,   16);
    bench_fill(b,   BENCH_MAX_N, BENCH_SEED_B,   16);
    bench_fill(key, BENCH_MAX_N, BENCH_SEED_KEY, 12);
    bench_fill(val, BENCH_MAX_N, BENCH_SEED_VAL, 16);
    bench_fill(x8,  BENCH_MAX_N, BENCH_SEED_X8,   8);
    bench_fill(x20, BENCH_MAX_N, BENCH_SEED_X20, 20);

    printf("\nAP (SoC at %u.%u MHz) vs Cortex-A9, same data, time per run\n",
           (unsigned)(clk_hz / 1000000), (unsigned)(clk_hz / 100000 % 10));
    printf("workload     n  AP cycles      AP us      A9 us   A9/AP  result\n");

    for (char *line = capture; line && *line; ) {
        char *next = strchr(line, '\n');
        if (next) *next++ = '\0';

        char name[16];
        unsigned n, cycles, result;
        if (sscanf(line, "BENCH %15s %u %u %u", name, &n, &cycles, &result) == 4) {
            uint32_t arm_result = arm_run(name, n);
            uint64_t arm_ns = arm_time_ns(name, n);
            uint64_t ap_ns = (uint64_t)cycles * 1000000000ull / clk_hz;
            int ok = (arm_result == result);
            if (!ok) mismatches++;
            printf("%-8s %5u %10u ", name, n, cycles);
            print_us(ap_ns);
            printf(" ");
            print_us(arm_ns);
            printf(" %4u.%02u  %s\n", (unsigned)(arm_ns / ap_ns),
                   (unsigned)(arm_ns * 100 / ap_ns % 100), ok ? "same" : "DIFFERENT");
        }
        line = next;
    }
    return mismatches;
}

int main(void)
{
    // The ARM must not cache the SoC's external memory (see above).
    Xil_DCacheFlushRange((UINTPTR)ext_mem, EXT_SIZE);
    for (uint32_t off = 0; off < EXT_SIZE; off += 0x100000u)
        Xil_SetTlbAttributes((UINTPTR)ext_mem + off, NORM_NONCACHE);

    printf("\nAP SoC host\n");
    uint32_t id = ap_rd(AP_ID);
    if (id != AP_ID_VALUE) {
        printf("no AP SoC at 0x%08x (ID reads 0x%08x): bitstream loaded?\n",
               (unsigned)AP_BASE, (unsigned)id);
        return 1;
    }
    uint32_t clk_hz = measure_clock_hz();
    printf("array %u rows x %u columns, SoC clock %u Hz, external memory at 0x%08x\n",
           (unsigned)ap_rd(AP_ROWS), (unsigned)ap_rd(AP_COLS), (unsigned)clk_hz,
           (unsigned)(uintptr_t)ext_mem);

    int failed = 0;
    for (int i = 0; i < AP_NUM_PROGRAMS; i++) {
        const struct ap_program *p = &ap_programs[i];
        if (run_program(p, clk_hz) != 0) failed++;
        if (!strcmp(p->name, "scaling")) failed += compare_scaling(clk_hz) ? 1 : 0;
    }

    printf("\n%s: %d of %d programs failed\n", failed ? "FAIL" : "PASS", failed, AP_NUM_PROGRAMS);
    return 0;
}
