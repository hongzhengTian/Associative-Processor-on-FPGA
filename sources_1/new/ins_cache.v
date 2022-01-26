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
    output reg [ISA_WIDTH - 1 : 0]      instruction,
    output reg [OPCODE_WIDTH - 1 : 0]   ins_valid,

    /* the interface to DDR interface */
    output reg                          ISA_read_req,
    output reg [DDR_ADDR_WIDTH -1 : 0]  ISA_read_addr,
    input wire [ISA_WIDTH - 1 : 0]      instruction_to_cache,
    input wire [9 : 0]                  rd_cnt_isa,
    input wire                          rd_burst_data_valid,
    output reg [9 : 0]                  isa_read_len
);

/* states */
localparam                              START = 4'd1;
localparam                              LOAD_INS = 4'd2;
localparam                              SENT_INS = 4'd3;

reg [15 : 0]                            tag_ins;
reg [ISA_WIDTH - 1 : 0]                 ins_cache [0 : ISA_DEPTH -1];
reg [ISA_WIDTH - 1 : 0]                 int_serve;

reg [3 : 0]                             st_next;
reg [3 : 0]                             st_cur;
reg                                     ins_cache_init;
reg [9 : 0]                             ins_load_cnt;
reg [9 : 0]                             rd_cnt_isa_reg;
reg                                     rd_burst_data_valid_delay;

reg [ISA_WIDTH - 1 : 0]                 instruction_tmp;
reg [OPCODE_WIDTH - 1 : 0]              ins_valid_tmp;
reg                                     rst_cache;
reg [9 : 0]                             load_times;

integer i;

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
        rd_cnt_isa_reg <= 0;
        ins_cache_init <= 0;
        load_times <= 0;
        int_serve <= 0;
        tag_ins <= 0;
        instruction_tmp <= 0;
        ins_valid_tmp <= 0;
    end
    else begin
        case (st_cur)
            LOAD_INS: begin
                tag_ins <= addr_ins;
                if (rd_cnt_isa >= isa_read_len) begin
                    rd_cnt_isa_reg <= rd_cnt_isa;
                    ins_cache_init <= 1;
                    load_times <= load_times + 1;
                end
            end
            SENT_INS: begin
                if ((addr_ins - tag_ins) < ISA_DEPTH + 1&& addr_ins != {{1'b1}, {{ADDR_WIDTH_MEM - 1}{1'b0}}}) begin
                    instruction_tmp <= ins_cache[addr_ins - tag_ins - 1];
                    ins_valid_tmp <= {OPCODE_WIDTH{1'b1}};
                end
                else if (addr_ins == {{1'b1}, {{ADDR_WIDTH_MEM - 1}{1'b0}}}) begin
                    instruction_tmp <= int_serve;
                    ins_valid_tmp <= {OPCODE_WIDTH{1'b1}};
                end
                else begin
                    instruction_tmp <= 0;
                    ins_valid_tmp <= 0;
                end
            end
            default:;
        endcase
    end
end

always @(posedge clk or negedge rst) begin
    if (!rst) begin
        isa_read_len <= 0;
    end
    else if (addr_ins > {{1'b1}, {{ADDR_WIDTH_MEM - 1}{1'b0}}}) begin
        isa_read_len <= INT_INS_DEPTH + 1;
    end
    else if (TOTAL_ISA_DEPTH - rd_cnt_isa_reg > ISA_DEPTH ) begin
        isa_read_len <= ISA_DEPTH;
    end
    else begin
        isa_read_len <= TOTAL_ISA_DEPTH - rd_cnt_isa_reg;
    end
end

always @(*) begin
    case (st_cur)
        START: begin
            ISA_read_req = 0;
            ISA_read_addr = 0;
            ins_load_cnt = 0;
            rst_cache = 0;
            instruction = instruction_tmp;
            ins_valid = ins_valid_tmp;
            if(ins_cache_init == 0) begin
                st_next = LOAD_INS;
                ins_cache_rdy = 0;
            end
            else begin
                ins_valid = 0;
                st_next = SENT_INS;
                ins_cache_rdy = 1;
            end
        end
        SENT_INS: begin
            ISA_read_req = 0;
            ISA_read_addr = 0;
            if ((addr_ins - tag_ins) < ISA_DEPTH + 1 && addr_ins != {{1'b1}, {{ADDR_WIDTH_MEM - 1}{1'b0}}}) begin
                instruction = ins_cache[addr_ins - tag_ins - 1];
                ins_valid = {OPCODE_WIDTH{1'b1}};
                st_next = START;
                rst_cache = 0;
                ins_cache_rdy = 0;
            end
            else if (addr_ins == {{1'b1}, {{ADDR_WIDTH_MEM - 1}{1'b0}}}) begin
                instruction = int_serve;
                ins_valid = {OPCODE_WIDTH{1'b1}};
                st_next = SENT_INS;
                rst_cache = 0;
                ins_cache_rdy = 1;
            end
            else begin
                st_next = LOAD_INS;
                rst_cache = 1;
                instruction = 0;
                ins_valid = 0;
                ins_cache_rdy = 0;
            end
        end
        LOAD_INS: begin
            instruction = instruction_tmp;
            ins_valid = ins_valid_tmp;
            rst_cache = 0;
            ins_cache_rdy = 0;
            ISA_read_req = 1;
            if (load_times <= 2) begin
                ISA_read_addr = addr_ins << 3;
            end
            else begin
                ISA_read_addr = (addr_ins - 1) << 3;
            end
            if (rd_cnt_isa < isa_read_len ) begin
                st_next = LOAD_INS;
            end
            else begin
                st_next = START;
                ISA_read_req = 0;
            end
        end
        default : begin
            st_next = START;
            ins_cache_rdy = 0;
            rst_cache = 0;
            ISA_read_req = 0;
            ISA_read_addr = 0; /* maybe wrong here when load ISA */
            instruction = 0;
            ins_valid = 0;
        end
    endcase
end

always @(posedge clk) begin
    rd_burst_data_valid_delay <= rd_burst_data_valid;
end

always @(posedge clk or negedge rst or posedge rst_cache) begin
    if (!rst || rst_cache) begin
        for (i = 0; i <= ISA_DEPTH; i = i + 1) begin
            ins_cache[i] <= 0;
        end
    end
    else if (st_cur == LOAD_INS && rd_burst_data_valid_delay == 1 && rd_cnt_isa >= 1) begin
        ins_cache[rd_cnt_isa - 1] <= instruction_to_cache;
    end
end
endmodule