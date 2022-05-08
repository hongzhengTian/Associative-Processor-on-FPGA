`define DDR_DATA_WIDTH          128
`define DDR_ADDR_WIDTH          28
`define DATA_WIDTH              8
`define DATA_DEPTH              16
`define ISA_DEPTH               128
`define DATA_CACHE_DEPTH        16
`define OPCODE_WIDTH            4
`define ADDR_WIDTH_CAM          8
`define OPRAND_2_WIDTH          2
`define ADDR_WIDTH_MEM          16
`define ISA_WIDTH               OPCODE_WIDTH + ADDR_WIDTH_CAM + OPRAND_2_WIDTH + ADDR_WIDTH_MEM 
`define TOTAL_ISA_DEPTH         234
`define CACHE_ISA_ADDR          10
`define CACHE_DATA_ADDR         10
`define TOTAL_DATA_DEPTH        128
`define INT_INS_DEPTH           28
