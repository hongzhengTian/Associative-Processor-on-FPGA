module ins_cache
#(
    parameter ISA_DEPTH         = 128,
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
    input wire                              clk,
    input wire                              rst,

    /* the interface of program counter */
    input wire [ADDR_WIDTH_MEM - 1 : 0]     addr_ins,
    output reg                              ins_cache_rdy,
    output  [3 : 0]                         st_cur_ins_cache,
    output reg [9 : 0]                      load_times,

    /* the interface of AP_ctrl */
    output reg [ISA_WIDTH - 1 : 0]          instruction,
    output reg [OPCODE_WIDTH - 1 : 0]       ins_valid,

    /* the interface to DDR interface */
    output reg                              ISA_read_req,
    output reg [DDR_ADDR_WIDTH -1 : 0]      ISA_read_addr,
    input wire [ISA_WIDTH - 1 : 0]          instruction_to_cache,
    input wire                              ddr_rdy,
    input wire [9 : 0]                      rd_cnt_isa,
    input wire                              rd_burst_data_valid,
    input wire [3 : 0]                      state_interface_module,
    output reg [9 : 0]                      isa_read_len
);

    /* op code */
    localparam                              RESET           = 4'd1;
    localparam                              RET             = 4'd2;
    localparam                              LOADRBR         = 4'd4;
    localparam                              LOADCBC         = 4'd5;
    localparam                              STORERBR        = 4'd6;
    localparam                              STORECBC        = 4'd7;
    localparam                              COPY            = 4'd8;
    localparam                              ADD             = 4'd9;
    localparam                              SUB             = 4'd10;
    localparam                              TSC             = 4'd11;
    localparam                              ABS             = 4'd12;

    /* operand 2 */
    localparam                              M_A             = 2'd1;
    localparam                              M_B             = 2'd2;
    localparam                              M_R             = 2'd3;

    localparam                              INS_CNT_WIDTH   = 10;

    /* states */
    localparam                              START           = 4'd1;
    localparam                              LOAD_INS        = 4'd2;
    localparam                              SENT_INS        = 4'd3;
    localparam                              MEM_READ_ISA	= 4'd5;

    reg [15 : 0]                            tag_ins;
    reg [ISA_WIDTH - 1 : 0]                 ins_cache [0 : ISA_DEPTH -1];
    reg [ISA_WIDTH - 1 : 0]                 int_serve [0 : 9];

    reg [3 : 0]                             st_next;
    reg [3 : 0]                             st_cur;
    reg                                     ins_cache_init;
    reg [INS_CNT_WIDTH - 1 : 0]             ins_load_cnt;
    //reg [9 : 0]                             isa_read_len;
    reg [9 : 0]                             rd_cnt_isa_reg;

    assign st_cur_ins_cache = st_cur;

    /* state machine */
    always @(posedge clk or negedge rst)
    begin
        if (!rst)
            begin
                st_cur          <= START;
                ins_cache_init  <= 0;
                ins_cache_rdy   <= 0;
                ins_cache [0]   <= 0;
                isa_read_len    <= 0;
                rd_cnt_isa_reg  <= 0;
                load_times      <= 0;
            end
        else
            begin
                st_cur          <= st_next;
            end    
    end

    always @(posedge clk)
    begin
        if (TOTAL_ISA_DEPTH - rd_cnt_isa_reg > ISA_DEPTH )
            begin
                isa_read_len <= ISA_DEPTH;
            end
        else isa_read_len <= TOTAL_ISA_DEPTH - rd_cnt_isa_reg;
    end

    always @(*)
    begin
        case (st_cur)
            START:
                begin
                    ISA_read_req = 0;
                    ins_load_cnt = 0;
                    if(ins_cache_init == 0)
                        begin
                            st_next = LOAD_INS;
                        end
                    else begin
                        ins_valid = 0;
                        ins_cache_rdy = 1;
                        if (addr_ins != 0)
                        begin
                            st_next = SENT_INS;
                        end
                        else st_next = START;
                    end
                end

            SENT_INS:
                begin
                    if ((addr_ins - tag_ins) < ISA_DEPTH && addr_ins < {{1'b1}, {{ADDR_WIDTH_MEM - 1}{1'b0}}})
                        begin
                            if(load_times <= 1)
                            instruction     = ins_cache[addr_ins - tag_ins];
                            else instruction     = ins_cache[addr_ins - tag_ins - 1];
                            ins_valid       = {OPCODE_WIDTH{1'b1}};
                            //ins_counter     = ins_counter + 1;
                            st_next         = START;
                        end
                    else if (addr_ins >= {{1'b1}, {{ADDR_WIDTH_MEM - 1}{1'b0}}})
                        begin
                            instruction     = int_serve[addr_ins - {{ADDR_WIDTH_MEM - 1}{1'b0}}];
                        end
                    else st_next            = LOAD_INS;
                end

            LOAD_INS:
                begin
                    ISA_read_req = 1;
                    tag_ins = addr_ins;
                    ISA_read_addr = {{(DDR_ADDR_WIDTH - ADDR_WIDTH_MEM){1'b0}}, addr_ins} * 8;///////
                    //if (ins_load_cnt <= ISA_DEPTH)
                    if (rd_cnt_isa < isa_read_len )//&& state_interface_module == MEM_READ_ISA)
                        begin
                            st_next = LOAD_INS;
                            
                        end
                    else begin
                        st_next = START;
                        ISA_read_req = 0;
                        ins_cache_init = 1;
                        load_times = load_times + 1;
                        rd_cnt_isa_reg =rd_cnt_isa;
                    end
                end
            
            default : ;
        endcase
    end

    always @(instruction_to_cache or st_cur or rd_cnt_isa or rd_burst_data_valid) 
    begin
        if (st_cur == LOAD_INS && rd_burst_data_valid == 1 && rd_cnt_isa >= 1)
        begin
            ins_cache_rdy = 0;
            ins_cache[rd_cnt_isa - 1] = instruction_to_cache;
            //ins_load_cnt = ins_load_cnt + 1;
        end
    end

endmodule