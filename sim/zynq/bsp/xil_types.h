// Stand-ins for the Xilinx standalone BSP headers used by the Zynq host
// program, so it can be compiled and partly tested without Vitis.
#ifndef XIL_TYPES_H
#define XIL_TYPES_H
#include <stdint.h>
typedef uintptr_t UINTPTR;
typedef uint32_t  u32;
typedef uint64_t  u64;
#endif
