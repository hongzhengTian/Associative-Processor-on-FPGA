// Smoke test: CPU, console, AP instructions through PCPI.
#include "runtime.h"
#include "ap.h"

int main(void)
{
    uint32_t v = ap_version();
    printf_("AP v%u.%u.%u: %u rows x %u columns\n",
            v >> 16, (v >> 8) & 0xff, v & 0xff, ap_rows(), ap_cols());

    ap_field_t a = AP_FIELD(0, 16), b = AP_FIELD(16, 16), s = AP_FIELD(32, 16);
    ap_write(a, 5, 1200);
    ap_write(b, 5, 34);
    ap_add(s, a, b);
    uint32_t r = ap_read(s, 5);
    printf_("1200 + 34 = %u\n", r);
    return r == 1234 ? 0 : 1;
}
