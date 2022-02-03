module ALU_data_cache
#(
    parameter DATA_CACHE_DEPTH  = 16,
    parameter DATA_WIDTH        = 16,
    parameter DATA_DEPTH        = 16,
    parameter DDR_ADDR_WIDTH    = 28,
    parameter ADDR_WIDTH_MEM    = 16,
    parameter ADDR_WIDTH_CAM    = 8
)
(
    input wire                              clk,
    input wire                              rst,
    input wire [ADDR_WIDTH_MEM - 1 : 0]     addr_cur_ctxt,
    input wire [ADDR_WIDTH_MEM - 1 : 0]     data_addr,
    input wire [15 :0]                      tag_data,
    input wire [9 : 0]                      rd_cnt_data,
    input wire [9 : 0]                      data_store_cnt,
    input wire                              rd_burst_data_valid_delay,
    input wire                              data_cmd_0,
    input wire                              store_ddr_en,

    output wire [ADDR_WIDTH_MEM - 1 : 0]    arith_1,
    output wire [DDR_ADDR_WIDTH - 1 : 0]    arith_2,
    output wire [ADDR_WIDTH_MEM - 1 : 0]    arith_3,
    output wire [DDR_ADDR_WIDTH - 1 : 0]    arith_4,
    output wire [9 : 0]                     arith_5,
    output wire [9 : 0]                     arith_6,
    output reg                              dc_exp_1,
    output wire                             dc_exp_2,
    output reg                              dc_exp_3,
    output wire                             dc_exp_4,
    output wire                             dc_exp_5,
    output wire                             dc_exp_7,
    output wire                             dc_exp_8,
    output wire                             dc_exp_9
);
wire dc_exp_1_1;
wire dc_exp_1_2;
wire dc_exp_3_1;
wire dc_exp_3_2;

assign arith_1 = addr_cur_ctxt + DATA_DEPTH + DATA_DEPTH + DATA_DEPTH;
assign arith_2 = data_addr << 3;
assign arith_3 = data_addr - tag_data;
assign arith_4 = tag_data << 3; 
assign arith_5 = rd_cnt_data - 2;
assign arith_6 = data_store_cnt + 1;

//assign dc_exp_1 = ((data_addr < tag_data) || (data_addr >= DATA_CACHE_DEPTH + tag_data))? 1 : 0;//!dc_exp_3;
assign dc_exp_1_1 = (data_addr < tag_data)? 1 : 0;
assign dc_exp_1_2 = (data_addr >= DATA_CACHE_DEPTH + tag_data)? 1 : 0;
assign dc_exp_2 = (rd_burst_data_valid_delay && rd_cnt_data == 1)? 1 : 0;
//assign dc_exp_3 = ((data_addr >= tag_data) && (data_addr < DATA_CACHE_DEPTH + tag_data))? 1 : 0;
assign dc_exp_3_1 = (data_addr >= tag_data)? 1 : 0;
assign dc_exp_3_2 = (data_addr < DATA_CACHE_DEPTH + tag_data)? 1 : 0;
assign dc_exp_4 = !store_ddr_en;
assign dc_exp_5 = (rd_cnt_data <= DATA_CACHE_DEPTH)? 1 : 0;
assign dc_exp_7 = (data_store_cnt < DATA_CACHE_DEPTH)? 1 : 0;
assign dc_exp_8 = !data_cmd_0;//(data_cmd == RowxRow_load || data_cmd == ColxCol_load || data_cmd == Addr_load)? 1 : 0;
assign dc_exp_9 = (rd_burst_data_valid_delay && rd_cnt_data >= 2)? 1 : 0;

always @(posedge clk or negedge rst) begin
    if (!rst) begin
        dc_exp_1 <= 0;
        dc_exp_3 <= 0;
    end
    else begin
        dc_exp_1 <= dc_exp_1_1 | dc_exp_1_2;
        dc_exp_3 <= dc_exp_3_1 & dc_exp_3_2;
    end
end

endmodule
