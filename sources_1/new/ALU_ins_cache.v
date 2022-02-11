module ALU_ins_cache
#(
    parameter ISA_DEPTH         = 128,
    parameter INT_INS_DEPTH     = 27,
    parameter DDR_ADDR_WIDTH    = 28,
    parameter OPCODE_WIDTH      = 4,
    parameter ADDR_WIDTH_CAM    = 8,
    parameter OPRAND_2_WIDTH    = 2,
    parameter ADDR_WIDTH_MEM    = 16,
    parameter TOTAL_ISA_DEPTH   = 128,
    parameter ISA_WIDTH         = OPCODE_WIDTH 
                                + ADDR_WIDTH_CAM
                                + OPRAND_2_WIDTH 
                                + ADDR_WIDTH_MEM
)
(
    input wire [9 : 0]                          load_times,
    input wire [ADDR_WIDTH_MEM - 1 : 0]         addr_ins,
    input wire [15 : 0]                         tag_ins,
    input wire [7 : 0]                          rd_cnt_ins_reg,
    input wire [7 : 0]                          rd_cnt_ins, 
    input wire [7 : 0]                          ins_read_len,
    input wire                                  st_cur_e_LI,
    input wire                                  rd_burst_data_valid,
    input wire                                  ddr_to_ic_empty_delay,

    output wire [9 : 0]                         arith_1,
    output wire [9 : 0]                         arith_2,
    output wire [9 : 0]                         arith_3,
    output wire [9 : 0]                         arith_4,
    output wire [DDR_ADDR_WIDTH -1 : 0]         arith_5,
    output wire [DDR_ADDR_WIDTH -1 : 0]         arith_6,
    output wire [9 : 0]                         arith_7,
    output wire                                 ic_exp_1,
    output wire                                 ic_exp_2,
    output wire                                 ic_exp_3,
    output wire                                 ic_exp_4,
    output wire                                 ic_exp_5,
    output wire                                 ic_exp_6
    );



assign arith_1 = load_times + 1;
assign arith_2 = addr_ins - tag_ins - 1;
assign arith_3 = INT_INS_DEPTH + 2;
assign arith_4 = TOTAL_ISA_DEPTH - rd_cnt_ins_reg;
assign arith_5 = addr_ins << 3;
assign arith_6 = (addr_ins - 1) << 3;
assign arith_7 = rd_cnt_ins - 1;

assign ic_exp_1= ((rd_cnt_ins < ins_read_len) || ddr_to_ic_empty_delay)? 1 : 0;
assign ic_exp_2= ((addr_ins < (tag_ins + ISA_DEPTH + 1)) && (addr_ins >= tag_ins))? 1 : 0;
assign ic_exp_3= (addr_ins == {{1'b1}, {{ADDR_WIDTH_MEM - 1}{1'b0}}})? 1 : 0;
assign ic_exp_4= (addr_ins > {{1'b1}, {{ADDR_WIDTH_MEM - 1}{1'b0}}})? 1 : 0;
assign ic_exp_5= (load_times <= 2)? 1 : 0;
assign ic_exp_6= (st_cur_e_LI && rd_burst_data_valid && rd_cnt_ins >= 1)? 1 : 0;

endmodule
