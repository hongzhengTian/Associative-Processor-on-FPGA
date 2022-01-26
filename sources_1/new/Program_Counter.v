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
    input wire                              int,

    /* the interface of AP_ctrl */
    input wire                              ret_valid,
    input wire                              ins_inp_valid,
    input wire [ADDR_WIDTH_MEM - 1 : 0]     ret_addr_pc,
    input wire                              ret_addr_pc_rdy,
    (* DONT_TOUCH = "1" *)input wire [DDR_ADDR_WIDTH - 1 : 0]     jmp_addr_pc,

    /* the interface of instruction cache */
    output reg [ADDR_WIDTH_MEM - 1 : 0]     addr_ins,
    input wire                              ins_cache_rdy
);

localparam                              START           = 4'd1;
localparam                              CNT_ADDR        = 4'd2;
localparam                              LOAD_JMP_ADDR   = 4'd3;
localparam                              LOAD_RET_ADDR   = 4'd4;
localparam                              LOAD_RET_END    = 4'd5;

reg [3 : 0]                             st_next;
reg [3 : 0]                             st_cur;

reg                                     tmp_ret_valid;
wire                                    ret_finish;
wire [ADDR_WIDTH_MEM - 1 : 0]           jmp_addr_pc_short;

assign jmp_addr_pc_short = jmp_addr_pc [ADDR_WIDTH_MEM - 1 : 0];

always @(posedge clk)
begin
    tmp_ret_valid <= ret_valid;
end

assign ret_finish = tmp_ret_valid & ~ret_valid;
/* ALU */
wire [ADDR_WIDTH_MEM - 1 : 0]           arith_1;
wire [ADDR_WIDTH_MEM - 1 : 0]           arith_2;
wire [ADDR_WIDTH_MEM - 1 : 0]           arith_3;
wire [ADDR_WIDTH_MEM - 1 : 0]           arith_4;

assign arith_1 = addr_ins + 1;
assign arith_2 = jmp_addr_pc >> 3;
assign arith_3 = {{1'b1}, {{ADDR_WIDTH_MEM - 1}{1'b0}}};
assign arith_4 = ret_addr_pc - 1;

/* state machine */
always @(posedge clk or negedge rst) begin
    if (!rst) begin
        st_cur <= START;
    end
    else begin
        st_cur <= st_next;
    end    
end

always @(*) begin
    case (st_cur)
        START: begin
            st_next = CNT_ADDR;
        end
        CNT_ADDR: begin
            case ({int, ret_valid})
                2'b10: st_next = LOAD_JMP_ADDR;
                2'b11: st_next = LOAD_JMP_ADDR;
                2'b01: st_next = LOAD_RET_ADDR;
                default: st_next = CNT_ADDR;
            endcase
        end
        LOAD_JMP_ADDR: begin
            case (ins_inp_valid)
                1'b1: st_next = CNT_ADDR;
                default: st_next = LOAD_JMP_ADDR;
            endcase
        end
        LOAD_RET_ADDR: begin
            case (ret_finish)
                1'b1: st_next = LOAD_RET_END;
                default: st_next = LOAD_RET_ADDR;
            endcase
        end
        LOAD_RET_END: begin
            case (ins_cache_rdy)
                1'b1: st_next = CNT_ADDR;
                default: st_next = LOAD_RET_END;
            endcase
        end
        default: st_next = START;
    endcase
end
    
    
always @(posedge clk or negedge rst) begin
    if (!rst) begin
        addr_ins <= 0;
    end
    else begin
        case (st_cur)
            CNT_ADDR: begin
                case ({ins_inp_valid, ret_valid, ins_cache_rdy})
                    3'b101: addr_ins <= arith_1;
                    default: ;
                endcase
            end
            LOAD_JMP_ADDR: begin
                case (ins_inp_valid)
                    1'b1: addr_ins <= arith_2;
                    default: addr_ins <= arith_3;
                endcase
            end
            LOAD_RET_ADDR: begin
                case (ret_addr_pc_rdy)
                    1'b1: addr_ins <= arith_4;
                    default: ;
                endcase
            end
            LOAD_RET_END: begin
                case (ins_cache_rdy)
                    1'b1: addr_ins <= arith_1;
                    default: ;
                endcase
            end 
            default:;
        endcase
    end
end
endmodule