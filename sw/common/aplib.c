#include "ap.h"

void ap_put_array(ap_field_t f, uint32_t first, const uint32_t *src, uint32_t n)
{
    for (uint32_t seg = 0; seg < n; seg += 32) {
        uint32_t k = (n - seg < 32) ? n - seg : 32;
        for (uint32_t i = 0; i < k; i++) ap_xwrite(i, src[seg + i]);
        ap_xput(f, first + seg, k);
    }
}

void ap_get_array(ap_field_t f, uint32_t first, uint32_t *dst, uint32_t n)
{
    for (uint32_t seg = 0; seg < n; seg += 32) {
        uint32_t k = (n - seg < 32) ? n - seg : 32;
        ap_xget(f, first + seg, k);
        for (uint32_t i = 0; i < k; i++) dst[seg + i] = ap_xread(i);
    }
}
