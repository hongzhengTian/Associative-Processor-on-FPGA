#ifndef XTIME_L_H
#define XTIME_L_H
#include "xil_types.h"
typedef u64 XTime;
#define COUNTS_PER_SECOND 333333333u
void XTime_GetTime(XTime *t);
#endif
