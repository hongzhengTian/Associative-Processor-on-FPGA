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

    /* the interface of AP_ctrl */
    input wire [DATA_WIDTH - 1 : 0]         data_out_rbr,
    input wire [DATA_DEPTH - 1 : 0]         data_out_cbc,
    input wire [ADDR_WIDTH_MEM - 1 : 0]     data_addr,
    input wire [2 : 0]                      data_cmd,
    input wire [ADDR_WIDTH_CAM - 1 : 0]     addr_cam_col,
    input wire                              store_ddr_en,

    output reg                              data_cache_rdy,
    output reg [DATA_WIDTH - 1 : 0]         data_in_rbr,
    output reg [DATA_DEPTH - 1 : 0]         data_in_cbc,
    output reg [ADDR_WIDTH_MEM - 1 : 0]     addr_cur_ctxt,

    /* the interface of DDR */
    output reg                              DATA_read_req,
    output reg                              DATA_store_req,
    output reg [DATA_WIDTH - 1 : 0]         DATA_to_ddr,
	output reg [DDR_ADDR_WIDTH - 1 : 0]		DATA_read_addr,
	output reg [DDR_ADDR_WIDTH - 1 : 0]		DATA_write_addr,
    input wire [DATA_WIDTH - 1 : 0]			DATA_to_cache,
    input wire [9 : 0]                      rd_cnt_data,
    input wire                              rd_burst_data_valid,
    input wire                              wr_burst_data_req,
    input wire [3 : 0]                      state_interface_module
);

/* states */
localparam                                  START           = 4'd1;
localparam                                  LOAD_DATA       = 4'd2;
localparam                                  STORE_DATA      = 4'd3;
localparam                                  SENT_DATA_RBR   = 4'd4;
localparam                                  SENT_DATA_CBC   = 4'd5;
localparam                                  GET_DATA_RBR    = 4'd6;
localparam                                  GET_DATA_CBC    = 4'd7;

/* data_cache command */
localparam                                  RowxRow_load    = 3'd1;
localparam                                  RowxRow_store   = 3'd2;
localparam                                  ColxCol_load    = 3'd3;
localparam                                  ColxCol_store   = 3'd4;

localparam                                  MEM_WRITE_DATA_STORE 	= 4'd9;

reg [15 :0]                                 tag_data;
reg [DATA_WIDTH - 1 : 0]                    data_cache [0 : DATA_CACHE_DEPTH - 1];
reg [9 : 0]                                 data_cnt;
reg [9 : 0]                                 data_store_cnt;

reg [3 : 0]                                 st_next;
reg [3 : 0]                                 st_cur;
reg                                         tag_store; /* indicate that the current tag is for store data */

integer j ;

/* state machine */
always @(posedge clk or negedge rst)
begin
    if (!rst)
        begin
            st_cur          <= START;
            data_cnt        <= 0;
            tag_store       <= 0;
            tag_data        <= 16'hFFFF;
            addr_cur_ctxt   <= 16'h5000;
        end
    else
        begin
            st_cur          <= st_next;
        end    
end

always @(*) 
begin
    case (st_cur)
        START:
            begin
                data_cache_rdy = 0;
                DATA_read_req  = 0;
                data_store_cnt  = 0;
                DATA_store_req = 0;
                DATA_to_ddr    = 0;
                if(data_cnt == DATA_CACHE_DEPTH - 1)
                begin
                    data_cnt = 0;
                end

                case (data_cmd)
                    RowxRow_load:   
                        begin
                            st_next     = SENT_DATA_RBR;
                        end
                    RowxRow_store:  
                        begin
                            st_next     = GET_DATA_RBR;
                            if ((tag_store == 1) && (data_addr <= tag_data))
                                begin
                                    tag_data    = data_addr;
                                end
                            else if((tag_store == 1) && (data_addr > tag_data))
                                begin
                                    tag_data    = tag_data;
                                end
                            else begin
                                tag_data    = data_addr;
                                tag_store   = 1;
                            end
                        end
                    ColxCol_load:   
                        begin
                            st_next     = SENT_DATA_CBC;
                        end
                    ColxCol_store:  
                        begin
                            st_next     = GET_DATA_CBC;
                            /*if ((tag_store == 1) && (data_addr <= tag_data))
                                begin
                                    tag_data    = data_addr;
                                end
                            else if((tag_store == 1) && (data_addr > tag_data))
                                begin
                                    tag_data    = tag_data;
                                end
                            else begin*/
                                tag_data    = data_addr;
                                tag_store   = 1;
                            //end
                        end
                    default:st_next     = START;   
                endcase
            end

        SENT_DATA_RBR:
            begin
                if (tag_data == 16'hFFFF)
                    begin
                        st_next         = LOAD_DATA;
                    end
                else if((data_addr - tag_data) < DATA_CACHE_DEPTH)
                    begin
                        data_cache_rdy  = 1;
                        data_in_rbr     = data_cache[data_addr - tag_data];
                        st_next         = START;
                    end

                else if ((data_addr - tag_data) >= DATA_CACHE_DEPTH)
                    begin
                        st_next         = LOAD_DATA;
                    end
                else    st_next         = SENT_DATA_RBR;
            end

        SENT_DATA_CBC:
            begin
                if (tag_data == 16'hFFFF)
                    begin
                        st_next         = LOAD_DATA;
                    end
                else if((data_addr - tag_data) < DATA_CACHE_DEPTH)
                    begin
                        data_cache_rdy  = 1;
                        for (j = 0; j <= DATA_CACHE_DEPTH - 1; j = j + 1) 
                        begin
                            data_in_cbc[j] = data_cache[j][addr_cam_col];
                        end 
                        st_next = START;
                    end
                
                else if ((data_addr - tag_data) >= DATA_CACHE_DEPTH)
                    begin
                        st_next         = LOAD_DATA;
                    end

                else st_next = SENT_DATA_CBC;
            end

        LOAD_DATA:
            begin
                DATA_read_req   = 1;
                tag_data        = data_addr;
                DATA_read_addr = {{(DDR_ADDR_WIDTH - ADDR_WIDTH_MEM){1'b0}}, data_addr} * 8;
                if (rd_cnt_data <= DATA_CACHE_DEPTH)
                    st_next     = LOAD_DATA;
                else begin
                    st_next     = START;
                end
            end

        GET_DATA_RBR:
            begin
                if(store_ddr_en == 0)
                    begin
                        data_cache_rdy                      = 1;
                        data_cache[data_addr - tag_data]    = data_out_rbr;
                        st_next                             = START;
                        data_cnt                            = data_cnt + 1;
                    end
                else if (store_ddr_en == 1)
                    begin
                        st_next                             = STORE_DATA;
                    end
                else    st_next                             = GET_DATA_RBR;
            end

        GET_DATA_CBC:
            begin
                if(store_ddr_en == 0)
                    begin
                        data_cache_rdy                      = 1;
                        for (j = 0; j <= DATA_CACHE_DEPTH - 1; j = j + 1) begin
                            data_cache[j][addr_cam_col] = data_out_cbc[j];
                        end
                        st_next = START;
                    end
                else if(store_ddr_en == 1)
                    begin
                        st_next                             = STORE_DATA;
                    end
                else    st_next                             = GET_DATA_CBC;
            end

        STORE_DATA:
            begin
                DATA_store_req  = 1;
                data_cache_rdy  = 0;
                DATA_write_addr = {{(DDR_ADDR_WIDTH - ADDR_WIDTH_MEM){1'b0}}, tag_data} * 8;
                if (data_store_cnt < DATA_CACHE_DEPTH)
                    st_next     = STORE_DATA;
                else begin
                    st_next     = START;
                end
            end
        default: st_next = START;
    endcase
end

always @(DATA_to_cache or st_cur or rd_burst_data_valid or rd_cnt_data) 
begin
    if(st_cur == LOAD_DATA && rd_burst_data_valid == 1 && rd_cnt_data >= 2)
        begin
            data_cache_rdy = 0;
            data_cache[rd_cnt_data - 2] = DATA_to_cache;
            //data_load_cnt = data_load_cnt + 1;
        end    
end

always @(posedge clk) 
begin
    if(st_cur == STORE_DATA && wr_burst_data_req && (state_interface_module == MEM_WRITE_DATA_STORE))
        begin
            DATA_to_ddr <= data_cache[data_store_cnt];
            data_store_cnt <= data_store_cnt + 1;
        end
    else begin
        DATA_to_ddr <= 0;
    end
end

endmodule