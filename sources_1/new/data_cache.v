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
    output reg                              data_to_ddr_rdy,
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


localparam                                  MEM_WRITE_DATA_STORE = 4'd9;

reg [15 :0]                                 tag_data;
reg [DATA_WIDTH - 1 : 0]                    data_cache [0 : DATA_CACHE_DEPTH - 1];
reg [9 : 0]                                 data_store_cnt;
reg [9 : 0]                                 data_store_cnt_tmp;

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

always @(posedge store_ctxt_finish or negedge rst) begin
    if (!rst) begin
        addr_cur_ctxt <= addr_init_ctxt;
    end
    else if (store_ctxt_finish == 1) begin
        addr_cur_ctxt <= addr_cur_ctxt + DATA_DEPTH + DATA_DEPTH + DATA_DEPTH;
    end
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
        data_in_rbr_tmp <= 0;
        data_in_cbc_tmp <= 0;
        DATA_read_addr_tmp <= 0;
        DATA_write_addr_tmp <= 0;
        data_store_cnt_tmp <= 0;
    end
    else begin
        case (st_cur)
            START:begin
                data_store_cnt_tmp <= 0;
                case (data_cmd)
                    RowxRow_store: begin
                        if ((data_addr <= tag_data) || (data_addr > DATA_CACHE_DEPTH + tag_data)) begin
                            tag_data <= data_addr;
                        end
                    end
                    ColxCol_store: begin
                        if (store_ddr_en == 0) begin
                            tag_data <= data_addr;
                        end
                    end
                    default:;
                endcase
            end
            SENT_ADDR: begin
                tag_data <= data_addr;
                DATA_read_addr_tmp <= data_addr << 3;
                if(rd_burst_data_valid_delay == 1 && rd_cnt_data == 1) begin
                    jmp_addr_tmp <= JMP_ADDR_to_cache;
                end 
                else begin
                    jmp_addr_tmp <= 0;
                end
            end
            SENT_DATA_RBR: begin
                if((data_addr - tag_data) < DATA_CACHE_DEPTH) begin
                    data_in_rbr_tmp     <= data_cache[data_addr - tag_data];
                end
            end
            SENT_DATA_CBC: begin
                if((data_addr - tag_data) < DATA_CACHE_DEPTH) begin
                    for (j = 0; j <= DATA_CACHE_DEPTH - 1; j = j + 1) begin
                        data_in_cbc_tmp[j] = data_cache[j][addr_cam_col];
                    end 
                end
            end
            LOAD_DATA: begin
                tag_data <= data_addr;
                DATA_read_addr_tmp <= data_addr << 3;
            end
            STORE_DATA: begin
                DATA_write_addr_tmp <= tag_data << 3; 
            end
            default:;
        endcase
    end
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
            case (data_cmd)
                RowxRow_load: st_next = SENT_DATA_RBR;
                RowxRow_store: st_next = GET_DATA_RBR;
                ColxCol_load: st_next = SENT_DATA_CBC;
                ColxCol_store: begin
                    if (store_ddr_en == 0) begin
                        st_next = GET_DATA_CBC;
                    end
                    else begin
                        st_next = STORE_DATA;
                    end
                end
                Addr_load: st_next = SENT_ADDR;
                default:begin
                    if (store_ddr_en == 0) begin
                        st_next = START;
                    end
                    else begin
                        st_next = STORE_DATA;
                    end
                end
            endcase
        end
        SENT_ADDR: begin
            JMP_ADDR_read_req = 1;
            DATA_read_req = 0;
            DATA_store_req = 0;
            data_in_rbr = 0;
            data_in_cbc = 0;
            DATA_read_addr = data_addr << 3;
            DATA_write_addr = DATA_write_addr_tmp;
            if(rd_burst_data_valid_delay == 1 && rd_cnt_data == 1) begin
                data_cache_rdy = 1;
                jmp_addr_rdy = 1;
                jmp_addr = JMP_ADDR_to_cache;
                st_next = START;
            end 
            else begin
                st_next = SENT_ADDR;
                data_cache_rdy = 0;
                jmp_addr_rdy = 0;
                jmp_addr = 0;
            end
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
            if (tag_data == 16'hFFFF) begin
                st_next = LOAD_DATA;
                data_cache_rdy = 0;
                data_in_rbr = data_in_rbr_tmp;
            end
            else if((data_addr - tag_data) < DATA_CACHE_DEPTH) begin
                data_cache_rdy = 1;
                data_in_rbr = data_cache[data_addr - tag_data];
                st_next = START;
            end
            else if ((data_addr - tag_data) >= DATA_CACHE_DEPTH && int_set == 0) begin
                st_next = LOAD_DATA;
                data_in_rbr = data_in_rbr_tmp;
                data_cache_rdy = 0;
            end
            else if (int_set == 1) begin
                st_next = GET_DATA_CBC;
                data_in_rbr = data_in_rbr_tmp;
                data_cache_rdy = 0;
            end
            else begin
                st_next = SENT_DATA_RBR;
                data_in_rbr = data_in_rbr_tmp;
                data_cache_rdy = 0;
            end
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
            if (tag_data == 16'hFFFF) begin
                st_next = LOAD_DATA;
                data_cache_rdy = 0;
                data_in_cbc = data_in_cbc_tmp;
            end
            else if((data_addr - tag_data) < DATA_CACHE_DEPTH) begin
                data_cache_rdy  = 1;
                for (j = 0; j <= DATA_CACHE_DEPTH - 1; j = j + 1) begin
                    data_in_cbc[j] = data_cache[j][addr_cam_col];
                end 
                st_next = START;
            end    
            else if ((data_addr - tag_data) >= DATA_CACHE_DEPTH && int_set == 0) begin
                st_next = LOAD_DATA;
                data_cache_rdy = 0;
                data_in_cbc = data_in_cbc_tmp;
            end
            else if (int_set == 1) begin
                st_next = GET_DATA_CBC;
                data_cache_rdy = 0;
                data_in_cbc = data_in_cbc_tmp;
            end
            else begin
                st_next = SENT_DATA_CBC;
                data_cache_rdy = 0;
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
            DATA_read_addr = data_addr << 3;
            DATA_write_addr = DATA_write_addr_tmp;
            data_in_rbr = 0;
            data_in_cbc = 0;
            if (rd_cnt_data <= DATA_CACHE_DEPTH) begin
                st_next = LOAD_DATA;
            end
            else begin
                st_next = START;
            end
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
            if(store_ddr_en == 0) begin
                data_cache_rdy = 1;
                st_next = START;
            end
            else if (store_ddr_en == 1) begin
                st_next = STORE_DATA;
                data_cache_rdy = 0;
            end
            else begin
                st_next = GET_DATA_RBR;
                data_cache_rdy = 0;
            end
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
            if(store_ddr_en == 0) begin
                data_cache_rdy = 1;
                st_next = START;
            end
            else if(store_ddr_en == 1) begin
                st_next = STORE_DATA;
                data_cache_rdy  = 0;
            end
            else begin
                st_next = GET_DATA_CBC;
                data_cache_rdy = 0;
            end
        end
        STORE_DATA: begin
            jmp_addr_rdy = 0;
            jmp_addr = jmp_addr_tmp;
            DATA_store_req = 1;
            DATA_read_req = 0;
            JMP_ADDR_read_req = 0;
            data_cache_rdy = 0;
            DATA_write_addr = tag_data << 3;
            DATA_read_addr = DATA_read_addr_tmp;
            data_in_rbr = 0;
            data_in_cbc = 0;
            if (data_store_cnt < DATA_CACHE_DEPTH) begin
                st_next = STORE_DATA;
            end
            else begin
                st_next = STORE_DATA_END;
            end
        end
        STORE_DATA_END: begin
            st_next = START;
            jmp_addr_rdy = 0;
            jmp_addr = jmp_addr_tmp;
            data_in_rbr = 0;
            data_in_cbc = 0;
            DATA_read_req = 0;
            DATA_store_req = 0;
            JMP_ADDR_read_req = 0;
            DATA_read_addr = DATA_read_addr_tmp;
            DATA_write_addr = DATA_write_addr_tmp;
            if (data_cmd == RowxRow_load || 
                data_cmd == ColxCol_load || 
                data_cmd == Addr_load) begin
                data_cache_rdy  = 0;
            end
            else begin
                data_cache_rdy  = 1;
            end
        end
        default: begin
            st_next = START;
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

always @(posedge clk) begin
    rd_burst_data_valid_delay <= rd_burst_data_valid;
end

always @(posedge clk) begin
    if(st_cur == LOAD_DATA && rd_burst_data_valid_delay == 1 && rd_cnt_data >= 2) begin
        data_cache[rd_cnt_data - 2] <= DATA_to_cache;
    end    
    else if (st_cur == GET_DATA_RBR) begin
        data_cache[data_addr - tag_data] <= data_out_rbr;
    end
    else if (st_cur == GET_DATA_CBC) begin
        for (j = 0; j <= DATA_CACHE_DEPTH - 1; j = j + 1) begin
            data_cache[j][addr_cam_col] <= data_out_cbc[j];
        end
    end
end

always @(posedge clk) begin
    if((st_cur == STORE_DATA) && (data_to_ddr_rdy == 1)) begin
        data_store_cnt <= data_store_cnt + 1;
    end
    else begin
        data_store_cnt <= 0;
    end
end

always @(posedge clk) begin
    if(st_cur == STORE_DATA) begin
        DATA_to_ddr <= data_cache[data_store_cnt];
        data_to_ddr_rdy <= 1;
    end
    else begin
        DATA_to_ddr <= 0;
        data_to_ddr_rdy <= 0;
    end
end 

endmodule