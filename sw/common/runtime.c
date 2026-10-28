#include <stdarg.h>
#include "runtime.h"

void putchar_(char c)
{
    IO_CONSOLE = (uint8_t)c;
}

void puts_(const char *s)
{
    while (*s) putchar_(*s++);
}

static void print_unsigned(uint32_t v, unsigned base, int width, char pad)
{
    char buf[12];
    int n = 0;
    do {
        unsigned d = v % base;
        buf[n++] = (char)(d < 10 ? '0' + d : 'a' + d - 10);
        v /= base;
    } while (v != 0);
    while (n < width--) putchar_(pad);
    while (n > 0) putchar_(buf[--n]);
}

// Supports %c %s %d %u %x and a zero/space padded width, e.g. %08x or %6u.
void printf_(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    for (; *fmt; fmt++) {
        if (*fmt != '%') {
            putchar_(*fmt);
            continue;
        }
        fmt++;
        char pad = ' ';
        int width = 0;
        if (*fmt == '0') {
            pad = '0';
            fmt++;
        }
        while (*fmt >= '0' && *fmt <= '9') width = width * 10 + (*fmt++ - '0');
        switch (*fmt) {
        case 'c': putchar_((char)va_arg(ap, int)); break;
        case 's': puts_(va_arg(ap, const char *)); break;
        case 'u': print_unsigned(va_arg(ap, uint32_t), 10, width, pad); break;
        case 'x': print_unsigned(va_arg(ap, uint32_t), 16, width, pad); break;
        case 'd': {
            int32_t v = va_arg(ap, int32_t);
            if (v < 0) {
                putchar_('-');
                print_unsigned((uint32_t)-v, 10, width > 0 ? width - 1 : 0, pad);
            } else {
                print_unsigned((uint32_t)v, 10, width, pad);
            }
            break;
        }
        case '%': putchar_('%'); break;
        case '\0': fmt--; break;
        default: putchar_('%'); putchar_(*fmt); break;
        }
    }
    va_end(ap);
}

void *memset(void *dst, int c, size_t n)
{
    uint8_t *d = dst;
    while (n--) *d++ = (uint8_t)c;
    return dst;
}

void *memcpy(void *dst, const void *src, size_t n)
{
    uint8_t *d = dst;
    const uint8_t *s = src;
    while (n--) *d++ = *s++;
    return dst;
}
