#ifndef XIL_MMU_H
#define XIL_MMU_H
#include "xil_types.h"
#define NORM_NONCACHE 0x11DE2
void Xil_SetTlbAttributes(UINTPTR addr, u32 attrib);
#endif
