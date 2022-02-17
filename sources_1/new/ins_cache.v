module ins_cache
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
    /* the interface of system signal */
    input wire                          clk,
    input wire                          rst,

    /* the interface of program counter */
    input wire [ADDR_WIDTH_MEM - 1 : 0] addr_ins,
    output reg                          ins_cache_rdy,

    /* the interface of AP_ctrl */
    output reg [ISA_WIDTH - 1 : 0]      ins_to_apctrl,
    output reg [OPCODE_WIDTH - 1 : 0]   ins_valid,

    /* the interface to DDR interface */
    output reg                          ins_read_req,
    input wire                          ins_reading,
    output reg                          rd_en_ddr_to_ic_fifo,
    output reg [DDR_ADDR_WIDTH -1 : 0]  ins_read_addr,
    input wire [ISA_WIDTH + 8 : 0]      ins_fifo_to_ic,
    input wire                          ddr_to_ic_fifo_empty,
    output wire [7 : 0]                 ins_read_len
);

/* states */
localparam                              START = 4'd1;
localparam                              LOAD_INS = 4'd2;
localparam                              SENT_INS = 4'd3;

wire [ISA_WIDTH - 1 : 0]                ins_to_cache;
wire [7 : 0]                            rd_cnt_ins;
wire                                    rd_burst_data_valid;

assign ins_to_cache = ins_fifo_to_ic[ISA_WIDTH + 8 : 9];
assign rd_cnt_ins = ins_fifo_to_ic[8 : 1];
assign rd_burst_data_valid = ins_fifo_to_ic[0 : 0];

reg [15 : 0]                            tag_ins;
reg [ISA_WIDTH - 1 : 0]                 ins_cache [0 : ISA_DEPTH -1];
//reg [ISA_WIDTH - 1 : 0]                 int_serve;

wire [3 : 0]                            st_cur;
reg                                     ins_cache_init;

reg [7 : 0]                             rd_cnt_ins_reg;
reg                                     ddr_to_ic_fifo_empty_delay;

reg [ISA_WIDTH - 1 : 0]                 ins_tmp;
reg [OPCODE_WIDTH - 1 : 0]              ins_valid_tmp;
reg                                     rst_cache;
reg [9 : 0]                             load_times;
wire                                    st_cur_e_LI;

integer i;

/* ALU */
wire [9 : 0]                            arith_1;
wire [9 : 0]                            arith_2;
wire [9 : 0]                            arith_3;
wire [9 : 0]                            arith_4;
wire [DDR_ADDR_WIDTH -1 : 0]            arith_5;
wire [DDR_ADDR_WIDTH -1 : 0]            arith_6;
wire [9 : 0]                            arith_7;
wire                                    ic_exp_1;
wire                                    ic_exp_2;
wire                                    ic_exp_3;
wire                                    ic_exp_4;
wire                                    ic_exp_5;
wire                                    ic_exp_6;

assign ins_read_len = (ic_exp_4)? arith_3 : ISA_DEPTH;
assign st_cur_e_LI = (st_cur == LOAD_INS)? 1 : 0;

 ALU_ins_cache #(
    ISA_DEPTH,
    INT_INS_DEPTH,
    DDR_ADDR_WIDTH,
    OPCODE_WIDTH, 
    ADDR_WIDTH_CAM, 
    OPRAND_2_WIDTH,
    ADDR_WIDTH_MEM,
    TOTAL_ISA_DEPTH
)ALU_ins_cache_u
(
    .load_times                 (load_times),
    .addr_ins                   (addr_ins),
    .tag_ins                    (tag_ins),
    .rd_cnt_ins_reg             (rd_cnt_ins_reg),
    .rd_cnt_ins                 (rd_cnt_ins),
    .ins_read_len               (ins_read_len),
    .st_cur_e_LI                (st_cur_e_LI),
    .rd_burst_data_valid        (rd_burst_data_valid),
    .ddr_to_ic_fifo_empty_delay (ddr_to_ic_fifo_empty_delay),
    .arith_1                    (arith_1),
    .arith_2                    (arith_2),
    .arith_3                    (arith_3),
    .arith_4                    (arith_4),
    .arith_5                    (arith_5),
    .arith_6                    (arith_6),
    .arith_7                    (arith_7),
    .ic_exp_1                   (ic_exp_1),
    .ic_exp_2                   (ic_exp_2),
    .ic_exp_3                   (ic_exp_3),
    .ic_exp_4                   (ic_exp_4),
    .ic_exp_5                   (ic_exp_5),
    .ic_exp_6                   (ic_exp_6)
);

sm_ins_cache #(
    ISA_DEPTH,
    INT_INS_DEPTH,
    DDR_ADDR_WIDTH,
    OPCODE_WIDTH, 
    ADDR_WIDTH_CAM, 
    OPRAND_2_WIDTH,
    ADDR_WIDTH_MEM,
    TOTAL_ISA_DEPTH
)sm_ins_cache_u
(
    .clk                        (clk),
    .rst                        (rst),
    .ins_cache_init             (ins_cache_init),
    .ic_exp_2                   (ic_exp_2),
    .ic_exp_3                   (ic_exp_3),
    .ic_exp_1                   (ic_exp_1),
    .st_cur                     (st_cur)
);

always @(posedge clk or negedge rst) begin
    if (!rst) begin
        rd_cnt_ins_reg <= 0;
        ins_cache_init <= 0;
        load_times <= 0;
        //int_serve <= 0;
        tag_ins <= 0;
        ins_tmp <= 0;
        ins_valid_tmp <= 0;
    end
    else begin
        case (st_cur)
            LOAD_INS: begin
                tag_ins <= addr_ins;
                case(ic_exp_1)
                    1'b1: begin
                        rd_cnt_ins_reg <= rd_cnt_ins;
                        ins_cache_init <= 1;
                    end
                    1'b0: begin
                        load_times <= arith_1;
                    end
                    default:;
                endcase
            end
            SENT_INS: begin
                case ({ic_exp_2, ic_exp_3})
                2'b10: begin
                    ins_tmp <= ins_cache[arith_2];
                    ins_valid_tmp <= {OPCODE_WIDTH{1'b1}};
                end
                2'b01: begin
                    ins_tmp <= 0;//int_serve;
                    ins_valid_tmp <= {OPCODE_WIDTH{1'b1}};
                end
                2'b11: begin
                    ins_tmp <= 0;//int_serve;
                    ins_valid_tmp <= {OPCODE_WIDTH{1'b1}};
                end
                default: begin
                    ins_tmp <= 0;
                    ins_valid_tmp <= 0;
                end
                endcase
            end
            default:;
        endcase
    end
end

always @(posedge st_cur_e_LI or posedge ins_reading) begin // TODO
    if (st_cur_e_LI && ins_reading) begin
        ins_read_req <= 0;
    end
    else begin
        ins_read_req <= 1;
    end
end

always @(*) begin
    case (st_cur)
        START: begin
            rd_en_ddr_to_ic_fifo = 0;
            ins_read_addr = 0;
            rst_cache = 0;
            ins_to_apctrl = ins_tmp;
            ins_cache_rdy = ins_cache_init;
            case (ins_cache_init)
                1'b1: ins_valid = 0;
                default: ins_valid = ins_valid_tmp;
            endcase
        end
        SENT_INS: begin
            rd_en_ddr_to_ic_fifo = 0;
            ins_read_addr = 0;
            case ({ic_exp_2, ic_exp_3})
                2'b10: begin
                    ins_to_apctrl = ins_cache[arith_2];
                    ins_valid = {OPCODE_WIDTH{1'b1}};
                    rst_cache = 0;
                    ins_cache_rdy = 0;
                end
                2'b01: begin
                    ins_to_apctrl = 0;//int_serve;
                    ins_valid = {OPCODE_WIDTH{1'b1}};
                    rst_cache = 0;
                    ins_cache_rdy = 1;
                end
                2'b11: begin
                    ins_to_apctrl = 0;//int_serve;
                    ins_valid = {OPCODE_WIDTH{1'b1}};
                    rst_cache = 0;
                    ins_cache_rdy = 1;
                end
                default: begin
                    rst_cache = 1;
                    ins_to_apctrl = 0;
                    ins_valid = 0;
                    ins_cache_rdy = 0;
                end
            endcase
        end
        LOAD_INS: begin
            ins_to_apctrl = ins_tmp;
            ins_valid = ins_valid_tmp;
            rst_cache = 0;
            ins_cache_rdy = 0;
            ins_read_addr = (ic_exp_5)? arith_5 : arith_6;
            rd_en_ddr_to_ic_fifo = ic_exp_1 && (!ddr_to_ic_fifo_empty);
        end
        default : begin
            ins_cache_rdy = 0;
            rst_cache = 0;
            rd_en_ddr_to_ic_fifo = 0;
            ins_read_addr = 0; /* maybe wrong here when load ISA */
            ins_to_apctrl = 0;
            ins_valid = 0;
        end
    endcase
end

always @(posedge clk) begin
    ddr_to_ic_fifo_empty_delay <= ddr_to_ic_fifo_empty;
end

always @(posedge clk or negedge rst or posedge rst_cache) begin
    if (!rst || rst_cache) begin
        for (i = 0; i <= ISA_DEPTH; i = i + 1) begin
            ins_cache[i] <= 0;
        end
    end
    else if (ic_exp_6) begin
        ins_cache[arith_7] <= ins_to_cache;
    end
end
endmodule