// Native test of the Zynq host program's benchmark comparison: feeds it the
// console output of the scaling benchmark from simulation and runs the ARM
// side of every workload on this machine. Every result must match.
//
//   host_check <scaling console output>
#include <stdio.h>
#include <time.h>

#define main host_main
#include "main.c"
#undef main

u32  Xil_In32(UINTPTR addr)                { (void)addr; return 0; }
void Xil_Out32(UINTPTR addr, u32 value)    { (void)addr; (void)value; }
void Xil_SetTlbAttributes(UINTPTR a, u32 b) { (void)a; (void)b; }
void Xil_DCacheFlushRange(UINTPTR a, u32 n) { (void)a; (void)n; }

void XTime_GetTime(XTime *t)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    *t = (u64)ts.tv_sec * COUNTS_PER_SECOND + (u64)ts.tv_nsec * (COUNTS_PER_SECOND / 1000000u) / 1000u;
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: host_check <scaling output>\n");
        return 2;
    }
    FILE *f = fopen(argv[1], "r");
    if (!f) {
        perror(argv[1]);
        return 2;
    }
    capture_len = (uint32_t)fread(capture, 1, sizeof capture - 1, f);
    capture[capture_len] = '\0';
    fclose(f);

    int lines = 0;
    for (const char *p = capture; (p = strstr(p, "BENCH ")) != NULL; p++) lines++;
    int mismatches = compare_scaling(50000000u);
    printf("%d benchmark lines, %d mismatches\n", lines, mismatches);
    if (lines == 0 || mismatches != 0) {
        printf("FAIL\n");
        return 1;
    }
    printf("PASS\n");
    return 0;
}
