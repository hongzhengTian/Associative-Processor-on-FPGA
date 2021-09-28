module int_stack
#(
    parameter DATA_WIDTH        = 16,
    parameter DATA_DEPTH        = 128,
    parameter ADDR_WIDTH_MEM    = 16,
    parameter STACK_DEPTH       = 8
)
(
    /* the interface of system signal */
    input wire                              clk,
    input wire                              rst,

    /* the interface of AP_ctrl */
    input wire                              int_set,    
    input wire                              ret_valid,
    input wire [ADDR_WIDTH_MEM - 1 : 0]     ret_addr,
    input wire [ADDR_WIDTH_MEM - 1 : 0]     ctxt_addr,
    input wire [DATA_WIDTH - 1 : 0]         tmp_bit_cnt,
    input wire [2 : 0]                      tmp_pass,
    input wire [DATA_WIDTH - 1 : 0]         tmp_mask,
    input wire [DATA_DEPTH - 1 : 0]         tmp_C_F,
    output reg                              ctxt_rdy,

    output reg [ADDR_WIDTH_MEM - 1 : 0]     ret_addr_ret,
    output reg [ADDR_WIDTH_MEM - 1 : 0]     ctxt_addr_ret,
    output reg [ADDR_WIDTH_MEM - 1 : 0]     ctxt_addr_A_ret,
    output reg [DATA_WIDTH - 1 : 0]         tmp_bit_cnt_ret,
    output reg [2 : 0]                      tmp_pass_ret,
    output reg [DATA_WIDTH - 1 : 0]         tmp_mask_ret,
    output reg [DATA_DEPTH - 1 : 0]         tmp_C_F_ret
);

localparam                                  START           = 3'd1;
localparam                                  STORE_CTXT      = 3'd2;
localparam                                  LOAD_CTXT       = 3'd3;
localparam                                  STACK_CNT_WIDTH = 10;

reg [2 : 0]                                 st_cur;
reg [2 : 0]                                 st_next;
reg [STACK_CNT_WIDTH : 0]                   stack_cnt;

reg [ADDR_WIDTH_MEM - 1 : 0]                stack_ret_addr      [0 : STACK_DEPTH - 1];
reg [ADDR_WIDTH_MEM - 1 : 0]                stack_ctxt_addr     [0 : STACK_DEPTH - 1];
reg [DATA_WIDTH - 1 : 0]                    stack_tmp_bit_cnt   [0 : STACK_DEPTH - 1];
reg [2 : 0]                                 stack_tmp_pass      [0 : STACK_DEPTH - 1];
reg [DATA_WIDTH - 1 : 0]                    stack_tmp_mask      [0 : STACK_DEPTH - 1];
reg [DATA_DEPTH - 1 : 0]                    stack_tmp_C_F       [0 : STACK_DEPTH - 1];

reg                                         temp_int_set;
reg                                         temp_ret_set;
wire                                        int_set_pause;
wire                                        ret_set_pause;

integer i;

always @(posedge clk) 
begin
    temp_int_set <= int_set;  
    temp_ret_set <= ret_valid;  
end

assign int_set_pause = ~temp_int_set & int_set;
assign ret_set_pause = ~temp_ret_set & ret_valid;

always @(posedge clk or negedge rst) 
begin
    if(!rst)
        begin
            for (i = 0; i <= STACK_DEPTH - 1; i = i + 1)
            begin
                stack_ret_addr[i]       <= 0;
                stack_ctxt_addr[i]      <= 0;
                stack_tmp_bit_cnt[i]    <= 0;
                stack_tmp_pass[i]       <= 0;
                stack_tmp_mask[i]       <= 0;
                stack_tmp_C_F[i]        <= 0;
            end
            ret_addr_ret    <= 0;
            ctxt_addr_ret   <= 0;
            ctxt_addr_A_ret <= 0;
            tmp_bit_cnt_ret <= 0;
            tmp_pass_ret    <= 0;
            tmp_mask_ret    <= 0;
            tmp_C_F_ret     <= 0;
        end
    case (st_cur)
        START:
        begin
            ctxt_rdy   <= 0;
        end
        STORE_CTXT:
        begin
            for (i = 0; i <= ADDR_WIDTH_MEM - 1; i = i + 1)
                begin
                    stack_ret_addr[stack_cnt - 1][i]    <= ret_addr[i];
                end

            for (i = 0; i <= ADDR_WIDTH_MEM - 1; i = i + 1)
                begin
                    stack_ctxt_addr[stack_cnt - 1][i]   <= ctxt_addr[i];
                end

            for (i = 0; i <= DATA_WIDTH - 1; i = i + 1)
                begin
                    stack_tmp_bit_cnt[stack_cnt - 1][i] <= tmp_bit_cnt[i];
                end

            for (i = 0; i <= 2; i = i + 1)
                begin
                    stack_tmp_pass[stack_cnt - 1][i]    <= tmp_pass[i];
                end

            for (i = 0; i <= DATA_WIDTH - 1; i = i + 1)
                begin
                    stack_tmp_mask[stack_cnt - 1][i]    <= tmp_mask[i];
                end

            for (i = 0; i <= DATA_DEPTH - 1; i = i + 1)
                begin
                    stack_tmp_C_F[stack_cnt - 1][i]     <= tmp_C_F[i];
                end
        end

    LOAD_CTXT:
        begin
            ctxt_rdy = 1;
            for (i = 0; i <= ADDR_WIDTH_MEM - 1; i = i + 1)
                begin
                    ret_addr_ret[i]     <= stack_ret_addr[stack_cnt][i];
                end

            for (i = 0; i <= ADDR_WIDTH_MEM - 1; i = i + 1)
                begin
                    ctxt_addr_ret[i]    <= stack_ctxt_addr[stack_cnt][i];
                end

            for (i = 0; i <= DATA_WIDTH - 1; i = i + 1)
                begin
                    tmp_bit_cnt_ret[i]  <= stack_tmp_bit_cnt[stack_cnt][i];
                end

            for (i = 0; i <= 2; i = i + 1)
                begin
                    tmp_pass_ret[i]     <= stack_tmp_pass[stack_cnt][i];
                end

            for (i = 0; i <= DATA_WIDTH - 1; i = i + 1)
                begin
                    tmp_mask_ret[i]     <= stack_tmp_mask[stack_cnt][i];
                end

            for (i = 0; i <= DATA_DEPTH - 1; i = i + 1)
                begin
                    tmp_C_F_ret[i]      <= stack_tmp_C_F[stack_cnt][i];
                end
        end

    default: 
        begin
            ret_addr_ret     <= 0;
            ctxt_addr_ret    <= 0;
            tmp_bit_cnt_ret  <= 0;
            tmp_pass_ret     <= 0;
            tmp_mask_ret     <= 0;
            tmp_C_F_ret      <= 0;
        end
    endcase
end

/* state machine */
always @(posedge clk or negedge rst)
begin
    if (!rst)
        begin
            st_cur          <= START;
            stack_cnt       <= 0;
        end
    else
        begin
            st_cur          <= st_next;
        end    
end

always @(st_cur)
begin
    if(st_cur == STORE_CTXT)
        stack_cnt = stack_cnt + 1;
    else if(st_cur == LOAD_CTXT)
        stack_cnt = stack_cnt -1 ;
end

always @(*)
begin
    case (st_cur)
        START:
            begin
                if(int_set_pause == 1)
                    begin
                        st_next = STORE_CTXT;
                    end
                else if (ret_set_pause == 1)
                    begin
                        st_next = LOAD_CTXT;
                    end
                else st_next =START;
            end

        STORE_CTXT:
            begin
                st_next = START;
            end

        LOAD_CTXT:
            begin
                st_next = START;
            end
        default: st_next = START;
    endcase
end

endmodule