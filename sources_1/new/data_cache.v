module data_cache
#(
    parameter DATA_CACHE_DEPTH  = 16,
    parameter DATA_WIDTH        = 16,
    parameter DATA_DEPTH        = 16,
    parameter DDR_ADDR_WIDTH    = 28,
    parameter ADDR_WIDTH_MEM    = 16,
    parameter ADDR_WIDTH_CAM    = 8
)
(
    /* the interface of system signal */
    input wire                              clk,
    input wire                              rst,
    input wire                              int_set,

    /* the interface of AP_ctrl */
    input wire [DATA_WIDTH - 1 : 0]         data_out_rbr,
    input wire [DATA_DEPTH - 1 : 0]         data_out_cbc,
    input wire [ADDR_WIDTH_MEM - 1 : 0]     data_addr,
    input wire [2 : 0]                      data_cmd,
    input wire [ADDR_WIDTH_CAM - 1 : 0]     addr_cam_col,
    input wire                              store_ddr_en,
    input wire                              store_ctxt_finish,

    output reg                              data_cache_rdy,
    output reg                              jmp_addr_rdy,
    output reg [DDR_ADDR_WIDTH - 1 : 0]		jmp_addr,
    output reg [DATA_WIDTH - 1 : 0]         data_in_rbr,
    output reg [DATA_DEPTH - 1 : 0]         data_in_cbc,
    output reg [ADDR_WIDTH_MEM - 1 : 0]     addr_cur_ctxt,

    /* the interface of DDR */
    output reg                              DATA_read_req,
    output reg                              DATA_store_req,
    output reg                              JMP_ADDR_read_req,
    input wire [DDR_ADDR_WIDTH - 1 : 0]		JMP_ADDR_to_cache,
    output reg [DATA_WIDTH - 1 : 0]         DATA_to_ddr,
	output reg [DDR_ADDR_WIDTH - 1 : 0]		DATA_read_addr,
	output reg [DDR_ADDR_WIDTH - 1 : 0]		DATA_write_addr,
    input wire [DATA_WIDTH - 1 : 0]			DATA_to_cache,
    input wire [9 : 0]                      rd_cnt_data,
    input wire                              rd_burst_data_valid
);

/* states */
localparam                                  START           = 4'd1;
localparam                                  LOAD_DATA       = 4'd2;
localparam                                  STORE_DATA      = 4'd3;
localparam                                  SENT_DATA_RBR   = 4'd4;
localparam                                  SENT_DATA_CBC   = 4'd5;
localparam                                  GET_DATA_RBR    = 4'd6;
localparam                                  GET_DATA_CBC    = 4'd7;
localparam                                  SENT_ADDR       = 4'd8;
localparam                                  LOAD_JMP_ADDR   = 4'd9;
localparam                                  STORE_DATA_END  = 4'd10;

/* data_cache command */
localparam                                  RowxRow_load    = 3'd1;
localparam                                  RowxRow_store   = 3'd2;
localparam                                  ColxCol_load    = 3'd3;
localparam                                  ColxCol_store   = 3'd4;
localparam                                  Addr_load       = 3'd5;

reg [15 :0]                                 tag_data;
reg [DATA_WIDTH - 1 : 0]                    data_cache [0 : DATA_CACHE_DEPTH - 1];
reg [9 : 0]                                 data_store_cnt;
reg                                         data_to_ddr_rdy;

reg [3 : 0]                                 st_next;
reg [3 : 0]                                 st_cur;
reg [ADDR_WIDTH_MEM - 1 : 0]                addr_init_ctxt = 16'h5000;
reg                                         rd_burst_data_valid_delay;
reg [DDR_ADDR_WIDTH - 1 : 0]		        jmp_addr_tmp;
reg [DATA_WIDTH - 1 : 0]                    data_in_rbr_tmp;
reg [DATA_DEPTH - 1 : 0]                    data_in_cbc_tmp;
reg [DDR_ADDR_WIDTH - 1 : 0]		        DATA_read_addr_tmp;
reg [DDR_ADDR_WIDTH - 1 : 0]		        DATA_write_addr_tmp;


integer j ;

/* ALU */
wire [ADDR_WIDTH_MEM - 1 : 0]               arith_1;
wire [DDR_ADDR_WIDTH - 1 : 0]               arith_2;
wire [ADDR_WIDTH_MEM - 1 : 0]               arith_3;
wire [DDR_ADDR_WIDTH - 1 : 0]               arith_4;
wire [9 : 0]                                arith_5;
wire [9 : 0]                                arith_6;
wire                                        dc_exp_1;
wire                                        dc_exp_2;
wire                                        dc_exp_3;
wire                                        dc_exp_4;
wire                                        dc_exp_5;
wire                                        dc_exp_6;
wire                                        dc_exp_7;
wire                                        dc_exp_8;
wire                                        dc_exp_9;
wire                                        dc_exp_10;

assign arith_1 = addr_cur_ctxt + DATA_DEPTH + DATA_DEPTH + DATA_DEPTH;
assign arith_2 = data_addr << 3;
assign arith_3 = data_addr - tag_data;
assign arith_4 = tag_data << 3; 
assign arith_5 = rd_cnt_data - 2;
assign arith_6 = data_store_cnt + 1;

assign dc_exp_1 = ((data_addr <= tag_data) || (data_addr > DATA_CACHE_DEPTH + tag_data))? 1 : 0;
assign dc_exp_2 = (rd_burst_data_valid_delay && rd_cnt_data == 1)? 1 : 0;
assign dc_exp_3 = ((data_addr - tag_data) < DATA_CACHE_DEPTH)? 1 : 0;
assign dc_exp_4 = ((data_addr - tag_data) >= DATA_CACHE_DEPTH && int_set == 0)? 1 : 0;
assign dc_exp_5 = (rd_cnt_data <= DATA_CACHE_DEPTH)? 1 : 0;
assign dc_exp_6 = (tag_data == 16'hFFFF)? 1 : 0;
assign dc_exp_7 = (data_store_cnt < DATA_CACHE_DEPTH)? 1 : 0;
assign dc_exp_8 = (data_cmd == RowxRow_load || 
                   data_cmd == ColxCol_load || 
                   data_cmd == Addr_load)? 1 : 0;
assign dc_exp_9 = (rd_burst_data_valid_delay && rd_cnt_data >= 2)? 1 : 0;

always @(posedge store_ctxt_finish or negedge rst) begin
    if (!rst) begin
        addr_cur_ctxt <= addr_init_ctxt;
    end
    else if (store_ctxt_finish) begin
        addr_cur_ctxt <= arith_1;
    end
end

always @(posedge clk) begin
    rd_burst_data_valid_delay <= rd_burst_data_valid;
end

/* state machine */
always @(posedge clk or negedge rst) begin
    if (!rst) begin
        st_cur <= START;
    end
    else begin
        st_cur <= st_next;
    end    
end

always @(posedge clk or negedge rst) begin
    if (!rst) begin
        tag_data <= 16'hFFFF;
        jmp_addr_tmp <= 0;
        DATA_to_ddr <= 0;
        data_to_ddr_rdy <= 0;
        data_store_cnt <= 0;
        data_in_rbr_tmp <= 0;
        data_in_cbc_tmp <= 0;
        DATA_read_addr_tmp <= 0;
        DATA_write_addr_tmp <= 0;
        for (j = 0; j <= DATA_CACHE_DEPTH - 1; j = j + 1) begin
            data_cache[j][addr_cam_col] <= 0;
        end
    end
    else begin
        case (st_cur)
            START:begin
                DATA_to_ddr <= 0;
                data_to_ddr_rdy <= 0;
                data_store_cnt <= 0;
                case (data_cmd)
                    RowxRow_store: begin
                        if (dc_exp_1) begin
                            tag_data <= data_addr;
                        end
                    end
                    ColxCol_store: begin
                        if (!store_ddr_en) begin
                            tag_data <= data_addr;
                        end
                    end
                    default:;
                endcase
            end
            SENT_ADDR: begin
                tag_data <= data_addr;
                DATA_read_addr_tmp <= arith_2;
                DATA_to_ddr <= 0;
                data_to_ddr_rdy <= 0;
                data_store_cnt <= 0;
                case (dc_exp_2)
                    1'b1:jmp_addr_tmp <= JMP_ADDR_to_cache; 
                    default: jmp_addr_tmp <= 0;
                endcase
            end
            SENT_DATA_RBR: begin
                DATA_to_ddr <= 0;
                data_to_ddr_rdy <= 0;
                data_store_cnt <= 0;
                if(dc_exp_3) begin
                    data_in_rbr_tmp <= data_cache[arith_3];
                end
            end
            SENT_DATA_CBC: begin
                DATA_to_ddr <= 0;
                data_to_ddr_rdy <= 0;
                data_store_cnt <= 0;
                if(dc_exp_3) begin
                    for (j = 0; j <= DATA_CACHE_DEPTH - 1; j = j + 1) begin
                        data_in_cbc_tmp[j] <= data_cache[j][addr_cam_col];
                    end 
                end
            end
            LOAD_DATA: begin
                tag_data <= data_addr;
                DATA_read_addr_tmp <= arith_2;
                DATA_to_ddr <= 0;
                data_to_ddr_rdy <= 0;
                data_store_cnt <= 0;
                if(dc_exp_9) begin
                    data_cache[arith_5] <= DATA_to_cache;
                end
            end
            STORE_DATA: begin
                DATA_write_addr_tmp <= arith_4; 
                DATA_to_ddr <= data_cache[data_store_cnt];
                data_to_ddr_rdy <= 1;
                if(data_to_ddr_rdy) begin
                    data_store_cnt <= arith_6;
                end
            end
            GET_DATA_RBR: begin
                data_cache[arith_3] <= data_out_rbr;
                DATA_to_ddr <= 0;
                data_to_ddr_rdy <= 0;
                data_store_cnt <= 0;
            end
            GET_DATA_CBC: begin
                DATA_to_ddr <= 0;
                data_to_ddr_rdy <= 0;
                data_store_cnt <= 0;
                for (j = 0; j <= DATA_CACHE_DEPTH - 1; j = j + 1) begin
                    data_cache[j][addr_cam_col] <= data_out_cbc[j];
                end
            end
            default:DATA_to_ddr <= 0;// check
        endcase
    end
end

/* state machine */
always @(*) begin
    case (st_cur)
        START: begin
            case ({data_cmd, store_ddr_en})
                {RowxRow_load, 1'b1}: st_next = SENT_DATA_RBR;
                {RowxRow_load, 1'b0}: st_next = SENT_DATA_RBR;
                {RowxRow_store, 1'b1}: st_next = GET_DATA_RBR;
                {RowxRow_store, 1'b0}: st_next = GET_DATA_RBR;
                {ColxCol_load, 1'b1}: st_next = SENT_DATA_CBC;
                {ColxCol_load, 1'b0}: st_next = SENT_DATA_CBC;
                {Addr_load, 1'b1}: st_next = SENT_ADDR;
                {Addr_load, 1'b0}: st_next = SENT_ADDR;
                {ColxCol_store, 1'b1}: st_next = STORE_DATA;
                {ColxCol_store, 1'b0}: st_next = GET_DATA_CBC;
                2'b01: st_next = STORE_DATA;
                2'b00: st_next = START;
                default: st_next = START;
            endcase
        end
        SENT_ADDR: begin
            case (dc_exp_2)
                1'b1: st_next = START; 
                default: st_next = SENT_ADDR;
            endcase
        end
        SENT_DATA_RBR: begin 
            case ({dc_exp_6, dc_exp_3, int_set})
                3'b100: st_next = LOAD_DATA;
                3'b110: st_next = LOAD_DATA;
                3'b010: st_next = START;
                3'b000: st_next = LOAD_DATA;
                3'b001: st_next = GET_DATA_CBC;
                3'b011: st_next = GET_DATA_CBC;
                3'b101: st_next = GET_DATA_CBC;
                3'b111: st_next = GET_DATA_CBC;
                default: st_next = SENT_DATA_RBR;
            endcase
        end
        SENT_DATA_CBC: begin
            case ({dc_exp_6, dc_exp_3, int_set})
                3'b100: st_next = LOAD_DATA;
                3'b110: st_next = LOAD_DATA;
                3'b010: st_next = START;
                3'b000: st_next = LOAD_DATA;
                3'b001: st_next = GET_DATA_CBC;
                3'b011: st_next = GET_DATA_CBC;
                3'b101: st_next = GET_DATA_CBC;
                3'b111: st_next = GET_DATA_CBC;
                default: st_next = SENT_DATA_CBC;
            endcase
        end
        LOAD_DATA: begin
            case (dc_exp_5)
                1'b1: st_next = LOAD_DATA; 
                default: st_next = START;
            endcase
        end
        GET_DATA_RBR: begin
            case (store_ddr_en)
                1'b0: st_next = START;
                1'b1: st_next = STORE_DATA;
                default: st_next = GET_DATA_RBR;
            endcase
        end
        GET_DATA_CBC: begin
            case (store_ddr_en)
                1'b0: st_next = START;
                1'b1: st_next = STORE_DATA;
                default: st_next = GET_DATA_CBC;
            endcase
        end
        STORE_DATA: begin
            case (dc_exp_7)
                1'b1: st_next = STORE_DATA; 
                default: st_next = STORE_DATA_END;
            endcase
        end
        STORE_DATA_END: begin
            st_next = START;
        end
        default: begin
            st_next = START;
        end
    endcase
end

always @(*) begin
    case (st_cur)
        START: begin
            data_cache_rdy = 0;
            jmp_addr_rdy = 0;
            jmp_addr = jmp_addr_tmp;
            DATA_read_req = 0;
            DATA_store_req = 0;
            JMP_ADDR_read_req = 0;
            data_in_rbr = data_in_rbr_tmp;
            data_in_cbc = data_in_cbc_tmp;
            DATA_read_addr = DATA_read_addr_tmp;
            DATA_write_addr = DATA_write_addr_tmp;
        end
        SENT_ADDR: begin
            JMP_ADDR_read_req = 1;
            DATA_read_req = 0;
            DATA_store_req = 0;
            data_in_rbr = 0;
            data_in_cbc = 0;
            DATA_read_addr = arith_2;
            DATA_write_addr = DATA_write_addr_tmp;
            data_cache_rdy = dc_exp_2;
            jmp_addr_rdy = dc_exp_2;
            jmp_addr = (dc_exp_2)? JMP_ADDR_to_cache : 0;
        end
        SENT_DATA_RBR: begin
            jmp_addr_rdy = 0;
            jmp_addr = jmp_addr_tmp;
            data_in_cbc = 0;
            DATA_read_req = 0;
            DATA_store_req = 0;
            JMP_ADDR_read_req = 0;
            DATA_read_addr = DATA_read_addr_tmp;
            DATA_write_addr = DATA_write_addr_tmp;
            data_cache_rdy = dc_exp_3;
            data_in_rbr = (dc_exp_3)? data_cache[arith_3] : data_in_rbr_tmp;
        end
        SENT_DATA_CBC: begin
            jmp_addr_rdy = 0;
            jmp_addr = jmp_addr_tmp;
            data_in_rbr = 0;
            DATA_read_req = 0;
            DATA_store_req = 0;
            JMP_ADDR_read_req = 0;
            DATA_read_addr = DATA_read_addr_tmp;
            DATA_write_addr = DATA_write_addr_tmp;
            data_cache_rdy = dc_exp_3;
            if(dc_exp_3) begin
                for (j = 0; j <= DATA_CACHE_DEPTH - 1; j = j + 1) begin
                    data_in_cbc[j] = data_cache[j][addr_cam_col];
                end 
            end    
            else begin
                data_in_cbc = data_in_cbc_tmp;
            end
        end
        LOAD_DATA: begin
            DATA_read_req = 1;
            DATA_store_req = 0;
            JMP_ADDR_read_req = 0;
            data_cache_rdy = 0;
            jmp_addr_rdy = 0;
            jmp_addr = jmp_addr_tmp;
            DATA_read_addr = arith_2;
            DATA_write_addr = DATA_write_addr_tmp;
            data_in_rbr = 0;
            data_in_cbc = 0;
        end
        GET_DATA_RBR: begin
            jmp_addr_rdy = 0;
            jmp_addr = jmp_addr_tmp;
            data_in_rbr = 0;
            data_in_cbc = 0;
            DATA_read_req = 0;
            DATA_store_req = 0;
            JMP_ADDR_read_req = 0;
            DATA_read_addr = DATA_read_addr_tmp;
            DATA_write_addr = DATA_write_addr_tmp;
            data_cache_rdy = !store_ddr_en;
        end
        GET_DATA_CBC: begin
            jmp_addr_rdy = 0;
            jmp_addr = jmp_addr_tmp;
            data_in_rbr = 0;
            data_in_cbc = 0;
            DATA_read_req = 0;
            DATA_store_req = 0;
            JMP_ADDR_read_req = 0;
            DATA_read_addr = DATA_read_addr_tmp;
            DATA_write_addr = DATA_write_addr_tmp;
            data_cache_rdy = !store_ddr_en;
        end
        STORE_DATA: begin
            jmp_addr_rdy = 0;
            jmp_addr = jmp_addr_tmp;
            DATA_store_req = 1;
            DATA_read_req = 0;
            JMP_ADDR_read_req = 0;
            data_cache_rdy = 0;
            DATA_write_addr = arith_4;
            DATA_read_addr = DATA_read_addr_tmp;
            data_in_rbr = 0;
            data_in_cbc = 0;
        end
        STORE_DATA_END: begin
            jmp_addr_rdy = 0;
            jmp_addr = jmp_addr_tmp;
            data_in_rbr = 0;
            data_in_cbc = 0;
            DATA_read_req = 0;
            DATA_store_req = 0;
            JMP_ADDR_read_req = 0;
            DATA_read_addr = DATA_read_addr_tmp;
            DATA_write_addr = DATA_write_addr_tmp;
            data_cache_rdy = !dc_exp_8;
        end
        default: begin
            data_cache_rdy = 0;
            jmp_addr_rdy = 0;
            jmp_addr = jmp_addr_tmp;
            data_in_cbc = 0;
            data_in_rbr = 0;
            DATA_store_req = 0;
            DATA_read_req = 0;
            JMP_ADDR_read_req = 0;
            DATA_read_addr = DATA_read_addr_tmp;
            DATA_write_addr = DATA_write_addr_tmp;
        end
    endcase
end
endmodule