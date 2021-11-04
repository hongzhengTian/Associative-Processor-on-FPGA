module program_counter
#(
    parameter ADDR_WIDTH_MEM    = 16,
    parameter ISA_DEPTH         = 64,
    parameter TOTAL_ISA_DEPTH   = 128,
    parameter DDR_ADDR_WIDTH    = 28
)
(
    /* the interface of system signal */
    input wire                              clk,
    input wire                              rst,
    input wire                              ret_valid,
    input wire                              int,

    /* the interface of AP_ctrl */
    input wire                              ins_inp_valid,
    input wire [ADDR_WIDTH_MEM - 1 : 0]     ret_addr_pc,
    input wire                              ret_addr_pc_rdy,
    (* DONT_TOUCH = "1" *)input wire [DDR_ADDR_WIDTH - 1 : 0]     jmp_addr_pc,
    output reg [ADDR_WIDTH_MEM - 1 : 0]     addr_cur_ins,
    input wire                              print_data_finish,

    /* the interface of instruction cache */
    output reg [ADDR_WIDTH_MEM - 1 : 0]     addr_ins,
    input wire                              ins_cache_rdy,
    input wire [3 : 0]                      st_cur_ins_cache,
    input wire [9 : 0]                      load_times
);
    integer i;

    localparam                              SENT_INS        = 4'd3;

    localparam                              START           = 4'd1;
    localparam                              CNT_ADDR        = 4'd2;
    localparam                              LOAD_JMP_ADDR   = 4'd3;
    localparam                              LOAD_RET_ADDR   = 4'd4;
    localparam                              LOAD_RET_END    = 4'd5;

    reg [3 : 0]                             st_next;
    reg [3 : 0]                             st_cur;

    reg                                     int_set;
    reg                                     tmp_ret_valid;
    wire                                    ret_finish;
    wire [ADDR_WIDTH_MEM - 1 : 0] jmp_addr_pc_short;
    assign jmp_addr_pc_short = jmp_addr_pc [ADDR_WIDTH_MEM - 1 : 0];
    always @(posedge clk)
    begin
        tmp_ret_valid <= ret_valid;
    end

    assign ret_finish = tmp_ret_valid & ~ret_valid;

    /* state machine */
    always @(posedge clk or negedge rst)
    begin
        if (!rst)
            begin
                st_cur          <= START;
            end
        else
            begin
                st_cur          <= st_next;
            end    
    end

    always @(posedge int or negedge rst or posedge ins_inp_valid)
    begin
        if (!rst)
        begin
            int_set <= 0;
        end
        else if (int == 1)
        begin
            int_set <= 1;    
        end
        else if (ins_inp_valid == 1)
        begin
            int_set <= 0;
        end
        else int_set <= 0;
    end

    always @(*) 
    begin
        case (st_cur)
            START:
                begin
                    st_next = CNT_ADDR;
                end
            CNT_ADDR:
                begin
                    if (int_set == 1)
                        begin
                            st_next = LOAD_JMP_ADDR;
                        end
                    else if (ret_valid == 1)
                        begin
                            st_next = LOAD_RET_ADDR;
                        end
                    else begin
                            st_next = CNT_ADDR;
                    end
                end
            LOAD_JMP_ADDR:
                begin
                    if (ins_inp_valid == 1)
                        begin
                            st_next = CNT_ADDR;
                        end
                    else st_next    = LOAD_JMP_ADDR;
                end
            LOAD_RET_ADDR:
                begin
                    if (ret_finish == 1)
                        begin
                            st_next = LOAD_RET_END;
                        end
                    else st_next    = LOAD_RET_ADDR;
                end
            LOAD_RET_END:
                begin
                    if (ins_inp_valid == 1)
                        begin
                            st_next = CNT_ADDR;
                        end
                    else st_next    = LOAD_RET_END;
                end
            default: st_next = START;
        endcase
    end
    
    
    always @(posedge clk or negedge rst) 
    begin
        if (!rst)
        begin
            addr_ins <= 0;
            addr_cur_ins <= 0;
        end

        else begin
        case (st_cur)
            CNT_ADDR:
                begin
                    if ((ins_inp_valid == 1) 
                    && (ret_valid == 0)
                    && (addr_ins < TOTAL_ISA_DEPTH) 
                    && (ins_cache_rdy == 1) 
                    &&(st_cur_ins_cache == SENT_INS)
                    &&(addr_ins != ISA_DEPTH * load_times)
                    )
                    begin
                        addr_ins <= addr_ins + 1;
                        addr_cur_ins <= addr_ins + 1;
                    end
                    else if ((print_data_finish == 1) 
                    && (ret_valid == 0)
                    && (addr_ins < TOTAL_ISA_DEPTH) 
                    && (ins_cache_rdy == 1) 
                    &&(addr_ins != ISA_DEPTH * load_times)
                    )
                    begin
                        addr_ins <= addr_ins + 1;
                        addr_cur_ins <= addr_ins + 1;
                    end
                    else
                    begin
                        addr_ins <= addr_ins;
                    end
                end
            LOAD_JMP_ADDR:
                begin
                    if (ins_inp_valid == 1)
                        begin
                            addr_ins <= jmp_addr_pc_short / 8;
                        end
                    else addr_ins <= {{1'b1}, {{ADDR_WIDTH_MEM - 1}{1'b0}}};
                end
            LOAD_RET_ADDR:
                begin
                    if (ret_addr_pc_rdy == 1)
                        begin
                            addr_ins <= ret_addr_pc;
                        end
                    else addr_ins <= addr_ins;
                end
            LOAD_RET_END:
                begin
                    if (ins_inp_valid == 1)
                        begin
                            addr_ins <= addr_ins + 1;
                        end
                    else addr_ins <= addr_ins;
                end 
            default:;
            endcase
            end
    end

endmodule