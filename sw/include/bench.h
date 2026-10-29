// Workloads of the scaling benchmark.
//
// Shared by the RISC-V program (sw/examples/scaling.c), which runs them on the
// AP unit, and the Zynq host program (fpga/zynq/host), which runs these C
// versions on the ARM core: both sides use the same data, generated here, and
// the same definition of every result.
//
// Each workload processes the first n elements and reduces its output to one
// 32-bit result, so the two sides can be compared without moving arrays.
#ifndef BENCH_H
#define BENCH_H

#include <stdint.h>

#define BENCH_MAX_N    1024

// data sets: seed and width in bits
#define BENCH_SEED_A   1u      // add: operand a, 16 bits
#define BENCH_SEED_B   2u      // add: operand b, 16 bits
#define BENCH_SEED_KEY 3u      // range: key, 12 bits
#define BENCH_SEED_VAL 4u      // range: value, 16 bits
#define BENCH_SEED_X8  5u      // hist16: sample, 8 bits
#define BENCH_SEED_X20 6u      // argmax, top8: value, 20 bits

// range query: BENCH_KEY_LO <= key < BENCH_KEY_HI and value > BENCH_VAL_MIN
#define BENCH_KEY_LO   1000u
#define BENCH_KEY_HI   3000u
#define BENCH_VAL_MIN  0x8000u

static inline uint32_t bench_rand(uint32_t *state)
{
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return *state = x;
}

// v[i] = i-th pseudo-random value of the given width, for i < n.
static inline void bench_fill(uint32_t *v, uint32_t n, uint32_t seed, uint32_t bits)
{
    uint32_t state = 0x9E3779B9u * seed;
    uint32_t mask = (bits >= 32) ? 0xffffffffu : (1u << bits) - 1u;
    for (uint32_t i = 0; i < n; i++) v[i] = bench_rand(&state) & mask;
}

// add: c[i] = a[i] + b[i]; result = sum of c.
static inline void bench_add(const uint32_t *a, const uint32_t *b, uint32_t *c, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) c[i] = a[i] + b[i];
}

static inline uint32_t bench_sum(const uint32_t *c, uint32_t n)
{
    uint32_t s = 0;
    for (uint32_t i = 0; i < n; i++) s += c[i];
    return s;
}

// range: number of elements inside the range query.
static inline uint32_t bench_range(const uint32_t *key, const uint32_t *val, uint32_t n)
{
    uint32_t count = 0;
    for (uint32_t i = 0; i < n; i++)
        if (key[i] >= BENCH_KEY_LO && key[i] < BENCH_KEY_HI && val[i] > BENCH_VAL_MIN) count++;
    return count;
}

// argmax: largest value and the lowest index holding it, as value | index << 20.
static inline uint32_t bench_argmax(const uint32_t *x, uint32_t n)
{
    uint32_t best = x[0], at = 0;
    for (uint32_t i = 1; i < n; i++)
        if (x[i] > best) { best = x[i]; at = i; }
    return best | (at << 20);
}

// hist16: histogram of x >> 4 (8-bit x); result = sum of (bin + 1) * count.
static inline uint32_t bench_hist16(const uint32_t *x, uint32_t n)
{
    uint32_t bins[16] = {0};
    for (uint32_t i = 0; i < n; i++) bins[x[i] >> 4]++;
    uint32_t s = 0;
    for (uint32_t b = 0; b < 16; b++) s += (b + 1) * bins[b];
    return s;
}

// top8: indexes of the 8 largest values, largest first, ties by lower index;
// result = sum of (rank + 1) * (index + 1).
static inline uint32_t bench_top8(const uint32_t *x, uint32_t n)
{
    uint32_t val[8], idx[8], k = 0;
    for (uint32_t i = 0; i < n; i++) {
        if (k == 8 && x[i] <= val[7]) continue;
        uint32_t j = (k < 8) ? k++ : 7;
        while (j > 0 && val[j - 1] < x[i]) {
            val[j] = val[j - 1];
            idx[j] = idx[j - 1];
            j--;
        }
        val[j] = x[i];
        idx[j] = i;
    }
    uint32_t s = 0;
    for (uint32_t j = 0; j < k; j++) s += (j + 1) * (idx[j] + 1);
    return s;
}

#endif
