module sm_ins_cache
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
    input wire                          clk,
    input wire                          rst,
    input wire                          ins_cache_init,
    input wire                          ic_exp_2,
    input wire                          ic_exp_3,
    input wire                          ic_exp_1,

    output reg [3 : 0]                  st_cur
);

/* states */
localparam                              START = 4'd1;
localparam                              LOAD_INS = 4'd2;
localparam                              SENT_INS = 4'd3;

reg [3 : 0]                             st_next;

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
            case (ins_cache_init)
                1'b1: st_next = SENT_INS; 
                default: st_next = LOAD_INS;
            endcase
        end
        SENT_INS: begin
            case ({ic_exp_2, ic_exp_3})
                2'b10: st_next = START;
                2'b01: st_next = SENT_INS;
                2'b11: st_next = SENT_INS;
                default: st_next = LOAD_INS;
            endcase
        end 
        LOAD_INS: begin
            case (ic_exp_1)
                1'b1: st_next = START; 
                default: st_next = LOAD_INS;
            endcase
        end
        default: st_next = START;
    endcase
end

endmodule
