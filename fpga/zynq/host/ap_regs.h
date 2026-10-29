// Register map of the AP SoC's host interface (rtl/zynq/ap_host_bridge.sv).
#ifndef AP_REGS_H
#define AP_REGS_H

// Address of the s_axi_ctrl port, as assigned by fpga/zynq/build.tcl.
#ifndef AP_BASE
#define AP_BASE          0x43C00000u
#endif

#define AP_CTRL          0x0000u     // bit 0 RUN
#define AP_STATUS        0x0004u
#define AP_EXIT          0x0008u
#define AP_CONSOLE       0x000Cu     // bit 8 valid, bits 7:0 byte; read pops
#define AP_RUN_CYCLES_LO 0x0010u
#define AP_RUN_CYCLES_HI 0x0014u
#define AP_CLK_COUNT     0x0018u
#define AP_ROWS          0x001Cu
#define AP_COLS          0x0020u
#define AP_EXT_BASE      0x0024u
#define AP_EXT_SIZE      0x0028u
#define AP_ID            0x002Cu
#define AP_RAM           0x10000u    // program RAM, 64 KiB, while RUN = 0

#define AP_STATUS_DONE    0x1u
#define AP_STATUS_TRAP    0x2u
#define AP_STATUS_CONSOLE 0x4u
#define AP_STATUS_RUN     0x8u

#define AP_CONSOLE_VALID  0x100u

#define AP_ID_VALUE       0x41500002u

#endif
