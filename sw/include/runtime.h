// Minimal bare-metal runtime for the AP SoC: console output, cycle counter,
// exit. No C library is used.
#ifndef RUNTIME_H
#define RUNTIME_H

#include <stdint.h>
#include <stddef.h>

#define IO_CONSOLE   (*(volatile uint32_t *)0x10000000u)
#define IO_EXIT      (*(volatile uint32_t *)0x10000004u)

// External memory window (AXI) as seen by the CPU.
#define EXT_MEM_BASE 0x80000000u

void putchar_(char c);
void puts_(const char *s);
void printf_(const char *fmt, ...);
void exit(int code) __attribute__((noreturn));

void *memset(void *dst, int c, size_t n);
void *memcpy(void *dst, const void *src, size_t n);

static inline uint32_t rdcycle(void)
{
    uint32_t c;
    __asm__ volatile ("rdcycle %0" : "=r"(c));
    return c;
}

// Small deterministic pseudo-random generator (xorshift32).
static inline uint32_t rand32(uint32_t *state)
{
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return *state = x;
}

#endif
