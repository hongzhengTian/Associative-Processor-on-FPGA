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
    output reg                              data_read_req,
    output reg                              data_store_req,
    output reg                              jmp_addr_read_req,
    //input wire [DDR_ADDR_WIDTH - 1 : 0]		jmp_addr_to_cache,
    output reg [DATA_WIDTH - 1 : 0]         data_to_ddr,
    output reg [9 : 0]                      wr_data_cnt_1,
	output reg [DDR_ADDR_WIDTH - 1 : 0]		data_read_addr,
	output reg [DDR_ADDR_WIDTH - 1 : 0]		data_write_addr,
    //input wire [DATA_WIDTH - 1 : 0]			data_to_cache,
    //input wire [9 : 0]                      rd_cnt_data,
    input wire [52 : 0]                     data_fifo_to_dc,
    output reg                              rd_en_ddr_to_dc_fifo,
    input wire                              ddr_to_dc_fifo_empty,
    input wire                              data_reading
    //input wire                              rd_burst_data_valid
);

/* states */
localparam                                  START_PRE       = 4'd11;
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
reg                                         wr_data_cnt_1_delay;
wire                                        wr_pulse;

wire [3 : 0]                                st_cur;
reg [ADDR_WIDTH_MEM - 1 : 0]                addr_init_ctxt = 16'h5000;

reg [DDR_ADDR_WIDTH - 1 : 0]		        jmp_addr_tmp;
reg [DATA_WIDTH - 1 : 0]                    data_in_rbr_tmp;
reg [DATA_DEPTH - 1 : 0]                    data_in_cbc_tmp;
reg [DDR_ADDR_WIDTH - 1 : 0]		        data_read_addr_tmp;
reg [DDR_ADDR_WIDTH - 1 : 0]		        data_write_addr_tmp;

wire [DDR_ADDR_WIDTH - 1 : 0]               jmp_addr_to_cache;
wire [DATA_WIDTH - 1 : 0]                   data_to_cache;
wire [7 : 0]                                rd_cnt_data;
wire                                        rd_burst_data_valid;
reg                                         ddr_to_dc_fifo_empty_delay;

wire                                        st_cur_e_LD;
wire                                        st_cur_e_START_PRE;
wire                                        st_cur_e_SENT_ADDR;
wire                                        st_cur_e_STORE_DATA;

integer j ;

assign data_to_cache = data_fifo_to_dc[DDR_ADDR_WIDTH + DATA_WIDTH + 8 : DDR_ADDR_WIDTH + 9];
assign jmp_addr_to_cache = data_fifo_to_dc[DDR_ADDR_WIDTH + 8 : 9];
assign rd_cnt_data = data_fifo_to_dc[8 : 1];
assign rd_burst_data_valid = data_fifo_to_dc[0 : 0];
assign st_cur_e_LD = (st_cur == LOAD_DATA)? 1 : 0;
assign st_cur_e_START_PRE = (st_cur == START_PRE)? 1 : 0;
assign st_cur_e_SENT_ADDR = (st_cur == SENT_ADDR)? 1 : 0;
assign st_cur_e_STORE_DATA = (st_cur == STORE_DATA)? 1 : 0;

/* ALU */
wire [ADDR_WIDTH_MEM - 1 : 0]               arith_1;
wire [DDR_ADDR_WIDTH - 1 : 0]               arith_2;
wire [ADDR_WIDTH_MEM - 1 : 0]               arith_3;
wire [DDR_ADDR_WIDTH - 1 : 0]               arith_4;
wire [7 : 0]                                arith_5;
wire [9 : 0]                                arith_6;
wire                                        dc_exp_1;
wire                                        dc_exp_2;
wire                                        dc_exp_3;
wire                                        dc_exp_4;
wire                                        dc_exp_5;
wire                                        dc_exp_7;
wire                                        dc_exp_8;
wire                                        dc_exp_9;

ALU_data_cache #(
    DATA_CACHE_DEPTH,
    DATA_WIDTH,
    DATA_DEPTH,
    DDR_ADDR_WIDTH,
    ADDR_WIDTH_MEM,
    ADDR_WIDTH_CAM
)
ALU_data_cache_u(
    .clk                        (clk),
    .rst                        (rst),
    .addr_cur_ctxt              (addr_cur_ctxt),
    .data_addr                  (data_addr),
    .tag_data                   (tag_data),
    .rd_cnt_data                (rd_cnt_data),
    .data_store_cnt             (data_store_cnt),
    .rd_burst_data_valid        (rd_burst_data_valid),
    .data_cmd_0                 (data_cmd[0]),
    .store_ddr_en               (store_ddr_en),
    .ddr_to_dc_fifo_empty_delay (ddr_to_dc_fifo_empty_delay),
    .arith_1                    (arith_1),
    .arith_2                    (arith_2),
    .arith_3                    (arith_3),
    .arith_4                    (arith_4),
    .arith_5                    (arith_5),
    .arith_6                    (arith_6),
    .dc_exp_1                   (dc_exp_1),
    .dc_exp_2                   (dc_exp_2),
    .dc_exp_3                   (dc_exp_3),
    .dc_exp_4                   (dc_exp_4),
    .dc_exp_5                   (dc_exp_5),
    .dc_exp_7                   (dc_exp_7),
    .dc_exp_8                   (dc_exp_8),
    .dc_exp_9                   (dc_exp_9)
);

sm_data_cache #(
    DATA_CACHE_DEPTH,
    DATA_WIDTH,
    DATA_DEPTH,
    DDR_ADDR_WIDTH,
    ADDR_WIDTH_MEM,
    ADDR_WIDTH_CAM
)
sm_data_cache_u(
    .clk                        (clk),
    .rst                        (rst),
    .int_set                    (int_set),
    .data_cmd                   (data_cmd),
    .store_ddr_en               (store_ddr_en),
    .dc_exp_2                   (dc_exp_2),
    .dc_exp_3                   (dc_exp_3),
    .dc_exp_5                   (dc_exp_5),
    .dc_exp_7                   (dc_exp_7),
    .st_cur                     (st_cur)
);

always @(posedge store_ctxt_finish or negedge rst) begin
    if (!rst) begin
        addr_cur_ctxt <= addr_init_ctxt;
    end
    else if (store_ctxt_finish) begin
        addr_cur_ctxt <= arith_1;
    end
end

always @(posedge clk) begin
    ddr_to_dc_fifo_empty_delay <= ddr_to_dc_fifo_empty;
end

always @ (posedge clk or rst) begin
	if(!rst) begin
		wr_data_cnt_1 <= 0;
	end
	else begin
		if (st_cur_e_STORE_DATA && !(wr_data_cnt_1 == DATA_CACHE_DEPTH - 1)) begin
			wr_data_cnt_1 <= wr_data_cnt_1 + 1;
		end
		else if (!st_cur_e_STORE_DATA) begin
			wr_data_cnt_1 <= 0;
		end
	end
end

always @(posedge clk) begin
	wr_data_cnt_1_delay <= wr_data_cnt_1[0 : 0];
end

assign wr_pulse = wr_data_cnt_1 ^ wr_data_cnt_1_delay;

always @(posedge st_cur_e_LD or posedge st_cur_e_START_PRE or posedge st_cur_e_SENT_ADDR or posedge data_reading) begin // TODO
    if (st_cur_e_LD && data_reading) begin
        data_read_req <= 0;
    end
    else if (st_cur_e_START_PRE) begin
        data_read_req <= 0;
    end
    else if (st_cur_e_SENT_ADDR) begin
        data_read_req <= 0;
    end
    else begin
        data_read_req <= 1;
    end
end

always @(posedge st_cur_e_SENT_ADDR or posedge st_cur_e_START_PRE or posedge data_reading) begin // TODO
    if (data_reading) begin
        jmp_addr_read_req <= 0;
    end
    else if (st_cur_e_SENT_ADDR) begin
        jmp_addr_read_req <= 1;
    end
    else begin
        jmp_addr_read_req <= 0;
    end
end

always @(posedge clk or negedge rst) begin
    if (!rst) begin
        tag_data <= 0;//16'hFFFF;
        jmp_addr_tmp <= 0;
        data_to_ddr <= 0;
        data_to_ddr_rdy <= 0;
        data_store_cnt <= 0;
        data_in_rbr_tmp <= 0;
        data_in_cbc_tmp <= 0;
        data_read_addr_tmp <= 0;
        data_write_addr_tmp <= 0;
        for (j = 0; j <= DATA_CACHE_DEPTH - 1; j = j + 1) begin
            data_cache[j][addr_cam_col] <= 0;
        end
    end
    else begin
        case (st_cur)
            START:begin
                data_to_ddr <= 0;
                data_to_ddr_rdy <= 0;
                data_store_cnt <= 0;
                case (data_cmd)
                    RowxRow_store: begin
                        if (dc_exp_1) begin
                            tag_data <= data_addr;
                        end
                    end
                    ColxCol_store: begin
                        if (dc_exp_4) begin
                            tag_data <= data_addr;
                        end
                    end
                    default:;
                endcase
            end
            SENT_ADDR: begin
                tag_data <= data_addr;
                data_read_addr_tmp <= arith_2;
                data_to_ddr <= 0;
                data_to_ddr_rdy <= 0;
                data_store_cnt <= 0;
                case (dc_exp_2)
                    1'b1:jmp_addr_tmp <= jmp_addr_to_cache; 
                    default: jmp_addr_tmp <= 0;
                endcase
            end
            SENT_DATA_RBR: begin
                data_to_ddr <= 0;
                data_to_ddr_rdy <= 0;
                data_store_cnt <= 0;
                if(dc_exp_3) begin
                    data_in_rbr_tmp <= data_cache[arith_3];
                end
            end
            SENT_DATA_CBC: begin
                data_to_ddr <= 0;
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
                data_read_addr_tmp <= arith_2;
                data_to_ddr <= 0;
                data_to_ddr_rdy <= 0;
                data_store_cnt <= 0;
                if(dc_exp_9) begin
                    data_cache[arith_5] <= data_to_cache;
                end
            end
            STORE_DATA: begin
                data_write_addr_tmp <= arith_4; 
                data_to_ddr_rdy <= 1;
                if(data_to_ddr_rdy) begin
                    data_to_ddr <= data_cache[data_store_cnt];
                    data_store_cnt <= arith_6;
                end
            end
            GET_DATA_RBR: begin
                data_cache[arith_3] <= data_out_rbr;
                data_to_ddr <= 0;
                data_to_ddr_rdy <= 0;
                data_store_cnt <= 0;
            end
            GET_DATA_CBC: begin
                data_to_ddr <= 0;
                data_to_ddr_rdy <= 0;
                data_store_cnt <= 0;
                for (j = 0; j <= DATA_CACHE_DEPTH - 1; j = j + 1) begin
                    data_cache[j][addr_cam_col] <= data_out_cbc[j];
                end
            end
            default:data_to_ddr <= 0;// check
        endcase
    end
end

always @(*) begin
    case (st_cur)
        START: begin
            rd_en_ddr_to_dc_fifo = 0;
            data_cache_rdy = 0;
            jmp_addr_rdy = 0;
            jmp_addr = jmp_addr_tmp;
            data_store_req = 0;
            data_in_rbr = data_in_rbr_tmp;
            data_in_cbc = data_in_cbc_tmp;
            data_read_addr = data_read_addr_tmp;
            data_write_addr = data_write_addr_tmp;
        end
        SENT_ADDR: begin
            rd_en_ddr_to_dc_fifo = !ddr_to_dc_fifo_empty;
            data_store_req = 0;
            data_in_rbr = 0;
            data_in_cbc = 0;
            data_read_addr = arith_2;
            data_write_addr = data_write_addr_tmp;
            data_cache_rdy = dc_exp_2;
            jmp_addr_rdy = dc_exp_2;
            //jmp_addr = (dc_exp_2)? jmp_addr_to_cache : 0;
            case (dc_exp_2)
                1'b1: jmp_addr = jmp_addr_to_cache;
                1'b0: jmp_addr = 0;
                default: jmp_addr = 0;
            endcase
        end
        SENT_DATA_RBR: begin
            rd_en_ddr_to_dc_fifo = 0;
            jmp_addr_rdy = 0;
            jmp_addr = jmp_addr_tmp;
            data_in_cbc = 0;
            data_store_req = 0;
            data_read_addr = data_read_addr_tmp;
            data_write_addr = data_write_addr_tmp;
            data_cache_rdy = dc_exp_3;
            //data_in_rbr = (dc_exp_3)? data_cache[arith_3] : data_in_rbr_tmp;
            case (dc_exp_3)
                1'b1: data_in_rbr = data_cache[arith_3];
                1'b0: data_in_rbr = data_in_rbr_tmp; 
                default: data_in_rbr = data_in_rbr_tmp;
            endcase
        end
        SENT_DATA_CBC: begin
            rd_en_ddr_to_dc_fifo = 0;
            jmp_addr_rdy = 0;
            jmp_addr = jmp_addr_tmp;
            data_in_rbr = 0;
            data_store_req = 0;
            data_read_addr = data_read_addr_tmp;
            data_write_addr = data_write_addr_tmp;
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
            rd_en_ddr_to_dc_fifo = !ddr_to_dc_fifo_empty;
            data_store_req = 0;
            data_cache_rdy = 0;
            jmp_addr_rdy = 0;
            jmp_addr = jmp_addr_tmp;
            data_read_addr = arith_2;
            data_write_addr = data_write_addr_tmp;
            data_in_rbr = 0;
            data_in_cbc = 0;
        end
        GET_DATA_RBR: begin
            rd_en_ddr_to_dc_fifo = 0;
            jmp_addr_rdy = 0;
            jmp_addr = jmp_addr_tmp;
            data_in_rbr = 0;
            data_in_cbc = 0;
            data_store_req = 0;
            data_read_addr = data_read_addr_tmp;
            data_write_addr = data_write_addr_tmp;
            data_cache_rdy = dc_exp_4;
        end
        GET_DATA_CBC: begin
            rd_en_ddr_to_dc_fifo = 0;
            jmp_addr_rdy = 0;
            jmp_addr = jmp_addr_tmp;
            data_in_rbr = 0;
            data_in_cbc = 0;
            data_store_req = 0;
            data_read_addr = data_read_addr_tmp;
            data_write_addr = data_write_addr_tmp;
            data_cache_rdy = dc_exp_4;
        end
        STORE_DATA: begin
            rd_en_ddr_to_dc_fifo = 0;
            jmp_addr_rdy = 0;
            jmp_addr = jmp_addr_tmp;
            data_store_req = 1;
            data_cache_rdy = 0;
            data_write_addr = arith_4;
            data_read_addr = data_read_addr_tmp;
            data_in_rbr = 0;
            data_in_cbc = 0;
        end
        STORE_DATA_END: begin
            rd_en_ddr_to_dc_fifo = 0;
            jmp_addr_rdy = 0;
            jmp_addr = jmp_addr_tmp;
            data_in_rbr = 0;
            data_in_cbc = 0;
            data_store_req = 0;
            data_read_addr = data_read_addr_tmp;
            data_write_addr = data_write_addr_tmp;
            data_cache_rdy = dc_exp_8;
        end
        default: begin
            rd_en_ddr_to_dc_fifo = 0;
            data_cache_rdy = 0;
            jmp_addr_rdy = 0;
            jmp_addr = jmp_addr_tmp;
            data_in_cbc = 0;
            data_in_rbr = 0;
            data_store_req = 0;
            data_read_addr = data_read_addr_tmp;
            data_write_addr = data_write_addr_tmp;
        end
    endcase
end
endmodule