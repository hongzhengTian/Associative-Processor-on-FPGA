module AP_controller
#(
    parameter DATA_WIDTH        = 16,
    parameter DATA_DEPTH        = 128,
    parameter OPCODE_WIDTH      = 4,
    parameter ADDR_WIDTH_CAM    = 8,
    parameter OPRAND_2_WIDTH    = 2,
    parameter ADDR_WIDTH_MEM    = 16,
    parameter DDR_ADDR_WIDTH    = 28,
    parameter ISA_WIDTH         = OPCODE_WIDTH 
                                + ADDR_WIDTH_CAM
                                + OPRAND_2_WIDTH 
                                + ADDR_WIDTH_MEM
)
(
    /* the interface of system signal */
    input wire                              clk,
    input wire                              rst_STATE,
    input wire                              rst_clk,
    input wire                              int,            /* interupt signal input */
    //input wire                              ret,            /* return signal input */
    output reg [5 : 0]                      st_cur,
    //output reg                              int_valid,
    output reg                              clk_d,

    /* the interface of instruction cache */
    input wire [OPCODE_WIDTH - 1 : 0]       ins_valid,
    input wire [ISA_WIDTH - 1 : 0]          instruction,

    /* the interface of data cache */
    input wire                              data_cache_rdy,
    input wire                              jmp_addr_rdy,
    input wire [DDR_ADDR_WIDTH - 1 : 0]		jmp_addr,
    input wire [DATA_WIDTH - 1 : 0]         data_in_rbr,
    input wire [DATA_DEPTH - 1 : 0]         data_in_cbc,
    input wire [ADDR_WIDTH_MEM - 1 : 0]     addr_cur_ctxt,
    output reg [ADDR_WIDTH_CAM - 1 : 0]     addr_cam_col,
    output reg [DATA_WIDTH - 1 : 0]         data_out_rbr,
    output reg [DATA_DEPTH - 1 : 0]         data_out_cbc,
    output reg [ADDR_WIDTH_MEM - 1 : 0]     data_addr,
    output reg [2 : 0]                      data_cmd,
    output wire                             store_ddr_en,
    output reg                              store_ctxt_finish,
    //output reg                              load_ctxt_finish,

    /* the interface of Program counter */
    input wire [ADDR_WIDTH_MEM - 1 : 0]     addr_cur_ins,
    output reg [DDR_ADDR_WIDTH - 1 : 0]     jmp_addr_pc,
    output reg                              ins_inp_valid,
    output reg [ADDR_WIDTH_MEM - 1 : 0]     ret_addr_pc,
    output reg                              ret_addr_pc_rdy,

    /* the interface of INT_STACK */
    /* temporary parameters from INT_STACK when RET signal set */
    input wire [ADDR_WIDTH_MEM - 1 : 0]     ret_addr_ret,
    input wire [ADDR_WIDTH_MEM - 1 : 0]     ctxt_addr_ret,
    input wire [DATA_WIDTH - 1 : 0]         tmp_bit_cnt_ret,
    input wire [2 : 0]                      tmp_pass_ret,
    input wire [DATA_WIDTH - 1 : 0]         tmp_mask_ret,
    input wire [DATA_DEPTH - 1 : 0]         tmp_C_F_ret,
    input wire                              tmp_key_A_ret,
    input wire                              tmp_key_B_ret,
    input wire                              tmp_key_C_ret,
    input wire                              tmp_key_F_ret,
    output reg                              int_set,    
    output reg                              ret_valid,
    output reg [ADDR_WIDTH_MEM - 1 : 0]     ret_addr,
    output reg [ADDR_WIDTH_MEM - 1 : 0]     ctxt_addr,
    output reg [DATA_WIDTH - 1 : 0]         tmp_bit_cnt,
    output reg [2 : 0]                      tmp_pass,
    output reg [DATA_WIDTH - 1 : 0]         tmp_mask,
    output reg [DATA_DEPTH - 1 : 0]         tmp_C_F,
    output reg                              tmp_key_A,
    output reg                              tmp_key_B,
    output reg                              tmp_key_C,
    output reg                              tmp_key_F,
    input wire                              ctxt_rdy,

    /* the interface of CAM */
    /* datas come from CAM */
    input wire [DATA_WIDTH - 1 : 0]         data_A_rbr,
    input wire [DATA_DEPTH - 1 : 0]         data_A_cbc,
    input wire [DATA_WIDTH - 1 : 0]         data_B_rbr,
    input wire [DATA_DEPTH - 1 : 0]         data_B_cbc,
    input wire [DATA_WIDTH - 1 : 0]         data_R_rbr,
    input wire [DATA_DEPTH - 1 : 0]         data_R_cbc,
    input wire [DATA_DEPTH - 1 : 0]         data_C,
    input wire [DATA_DEPTH - 1 : 0]         data_F,

    /* datas send to CAM */
    output reg [DATA_WIDTH - 1 : 0]         input_A_rbr,
    output reg [DATA_DEPTH - 1 : 0]         input_A_cbc,
    output reg [ADDR_WIDTH_CAM - 1 : 0]     addr_input_rbr_A,
    output reg [ADDR_WIDTH_CAM - 1 : 0]     addr_input_cbc_A,
    output reg [ADDR_WIDTH_CAM - 1 : 0]     addr_output_rbr_A,
    output reg [ADDR_WIDTH_CAM - 1 : 0]     addr_output_cbc_A,
    
    output reg [DATA_WIDTH - 1 : 0]         input_B_rbr,
    output reg [DATA_DEPTH - 1 : 0]         input_B_cbc,
    output reg [ADDR_WIDTH_CAM - 1 : 0]     addr_input_rbr_B,
    output reg [ADDR_WIDTH_CAM - 1 : 0]     addr_input_cbc_B,
    output reg [ADDR_WIDTH_CAM - 1 : 0]     addr_output_rbr_B,
    output reg [ADDR_WIDTH_CAM - 1 : 0]     addr_output_cbc_B,

    output reg [DATA_WIDTH - 1 : 0]         input_R_rbr,
    output reg [DATA_DEPTH - 1 : 0]         input_R_cbc,
    output reg [ADDR_WIDTH_CAM - 1 : 0]     addr_input_rbr_R,
    output reg [ADDR_WIDTH_CAM - 1 : 0]     addr_input_cbc_R,
    output reg [ADDR_WIDTH_CAM - 1 : 0]     addr_output_rbr_R,
    output reg [ADDR_WIDTH_CAM - 1 : 0]     addr_output_cbc_R,

    output reg [DATA_DEPTH - 1 : 0]         input_C,

    output reg [DATA_DEPTH - 1 : 0]         input_F,

    /* control signals of CAM */
    output reg                              ABS_opt,
    output reg                              rst_InA,
    output reg                              rst_InB,
    output reg                              rst_InC,
    output reg                              rst_InF,
    output reg                              rst_InR,
    output reg                              rst_tag,
    output reg [2 : 0]                      inout_mode,

    /* key, mask and pass information to CAM */
    output reg                              key_A,
    output reg                              key_B,
    output reg                              key_C,
    output reg                              key_F,
    output reg [DATA_WIDTH - 1 : 0]         mask,
    output reg                              mask_C,
    output reg                              mask_F,
    output reg [2 : 0]                      pass
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

    /* clock divider */
    localparam                              NUM_DIV         = 2;

    /* states */
    localparam                              START           = 6'd1;
    localparam                              LOAD_RBR        = 6'd2;
    localparam                              LOAD_CBC        = 6'd3;
    localparam                              COPY_MT         = 6'd4;
    localparam                              STORE_RBR       = 6'd5;
    localparam                              STORE_CBC       = 6'd6;
    localparam                              PASS_1_ADD      = 6'd7;
    localparam                              PASS_2_ADD      = 6'd8;
    localparam                              PASS_3_ADD      = 6'd9;
    localparam                              PASS_4_ADD      = 6'd10;
    localparam                              RSTTAG_ADD      = 6'd11;
    
    localparam                              PASS_1_SUB      = 6'd12;
    localparam                              PASS_2_SUB      = 6'd13;
    localparam                              PASS_3_SUB      = 6'd14;
    localparam                              PASS_4_SUB      = 6'd15;
    localparam                              RSTTAG_SUB      = 6'd16;
    
    localparam                              PASS_1_ABS      = 6'd17;
    localparam                              PASS_2_ABS      = 6'd18;
    localparam                              PASS_3_ABS      = 6'd19;
    localparam                              PASS_4_ABS      = 6'd20;
    localparam                              RSTTAG_ABS      = 6'd21;
    
    localparam                              PASS_1_TSC      = 6'd22;
    localparam                              PASS_2_TSC      = 6'd23;
    localparam                              PASS_3_TSC      = 6'd24;
    localparam                              RSTTAG_TSC      = 6'd25;

    localparam                              FINISH_CK       = 6'd26;
    localparam                              LOAD_TMP        = 6'd27;
    localparam                              LOAD_CTXT       = 6'd28;
    localparam                              LOAD_CTXT_FINISH_CHECK = 6'd29;
    localparam                              STORE_TMP       = 6'd30;
    localparam                              STORE_CTXT      = 6'd31;
    localparam                              STORE_CTXT_FINISH_CHECK = 6'd32;
    localparam                              GET_JMP_ADDR    = 6'd33;
    localparam                              JMP_INS         = 6'd34;
    localparam                              RET_STATE       = 6'd35;

    /* inout_mode */
    localparam                              RowxRow         = 3'd1;
    localparam                              ColxCol         = 3'd2;
    localparam                              COPY_B          = 3'd3;
    localparam                              COPY_R          = 3'd4;
    localparam                              COPY_A          = 3'd5;

    /* data_cache command */
    localparam                              RowxRow_load    = 3'd1;
    localparam                              RowxRow_store   = 3'd2;
    localparam                              ColxCol_load    = 3'd3;
    localparam                              ColxCol_store   = 3'd4;
    localparam                              Addr_load       = 3'd5;

    /* state variables */
    
    reg [5 : 0]                             st_next;
    reg [OPCODE_WIDTH - 1 : 0]              opt_cur;
    
    reg [DATA_WIDTH - 1 : 0]                bit_cnt;
    reg [2 : 0]                             clk_cnt;

    reg [2 : 0]                             pass_tmp;
    reg                                     key_A_tmp;
    reg                                     key_B_tmp;
    reg                                     key_C_tmp;
    reg                                     key_F_tmp;
    reg [ADDR_WIDTH_CAM - 1 : 0]            addr_cam_auto_tmp;
    reg [1 : 0]                             matrix_cnt_tmp;

    /* op_code and operands */
    wire [OPCODE_WIDTH - 1 : 0]             op_code;
    wire [OPCODE_WIDTH - 1 : 0]             op_code_valid;
    wire [ADDR_WIDTH_CAM - 1 : 0]           addr_cam;
    wire [OPRAND_2_WIDTH - 1 : 0]           matrix_select;
    reg  [OPRAND_2_WIDTH - 1 : 0]           matrix_select_reg;
    wire [OPRAND_2_WIDTH - 1 : 0]           matrix_select_1;
    wire [ADDR_WIDTH_MEM - 1 : 0]           addr_mem;
    
    reg [ADDR_WIDTH_MEM - 1 : 0]            addr_mem_col;
    reg [1 : 0]                             matrix_cnt;

    reg                                     tmp_store_ddr_en;
    reg                                     store_ddr_en_reg;
    reg [ADDR_WIDTH_CAM - 1 : 0]            addr_cam_auto;
    reg                                     tag_C_F; /* indicate which one do we store when interrupt*/
                                                     /* 1 means C, 0 means F */
    reg [DATA_WIDTH - 1 : 0]                data_out_rbr_tmp;
    reg [DATA_DEPTH - 1 : 0]                data_out_cbc_tmp;
    
    assign  store_ddr_en = tmp_store_ddr_en & ~store_ddr_en_reg;
    

    assign                                  op_code = instruction [OPCODE_WIDTH 
                                                                + ADDR_WIDTH_CAM
                                                                + OPRAND_2_WIDTH 
                                                                + ADDR_WIDTH_MEM - 1 :
                                                                + ADDR_WIDTH_CAM
                                                                + OPRAND_2_WIDTH 
                                                                + ADDR_WIDTH_MEM];
    assign                                  addr_cam = instruction [ADDR_WIDTH_CAM
                                                                + OPRAND_2_WIDTH 
                                                                + ADDR_WIDTH_MEM - 1 :
                                                                + OPRAND_2_WIDTH 
                                                                + ADDR_WIDTH_MEM];
    assign                                  matrix_select = instruction [OPRAND_2_WIDTH 
                                                                + ADDR_WIDTH_MEM - 1 : 
                                                                + ADDR_WIDTH_MEM];
    assign                                  matrix_select_1 = addr_cam [OPRAND_2_WIDTH - 1 : 0];
    assign                                  addr_mem = instruction [ADDR_WIDTH_MEM - 1 : 0];
    assign                                  op_code_valid = op_code & ins_valid;                                         
    
    /* clock divider */
    always @(posedge clk or negedge rst_clk)
    begin
        if (!rst_clk)
            begin
                clk_cnt <= 0;
                clk_d <= 0;
            end
        else if(clk_cnt < NUM_DIV / 2  - 1)
            begin
                clk_cnt <= clk_cnt + 1;
                clk_d <= clk_d;
            end
        else 
            begin
                clk_cnt <= 0;
                clk_d <= ~clk_d;
            end
    end

    always@(st_cur)
    begin
        if (st_cur == START)
        begin
            addr_cam_auto   = 0;
            matrix_cnt      = 1;
        end

        else if(st_cur == STORE_CTXT_FINISH_CHECK)
            begin
                if (addr_cam_auto < DATA_WIDTH )
                    begin
                        addr_cam_auto = addr_cam_auto + 1;
                        matrix_cnt = matrix_cnt_tmp;
                    end
                
                else if (addr_cam_auto == DATA_WIDTH && data_cache_rdy == 1)
                    begin
                        addr_cam_auto = 0;
                        matrix_cnt = matrix_cnt + 1;
                    end
                else begin
                    addr_cam_auto = addr_cam_auto_tmp;
                    matrix_cnt = matrix_cnt_tmp;
                end
            end
        else if(st_cur == LOAD_CTXT_FINISH_CHECK)
            begin
                if (addr_cam_auto < DATA_WIDTH - 1)
                    begin
                        addr_cam_auto = addr_cam_auto + 1;
                        matrix_cnt = matrix_cnt_tmp;
                    end
                
                else if (addr_cam_auto == DATA_WIDTH - 1 && data_cache_rdy == 1)
                    begin
                        addr_cam_auto = 0;
                        matrix_cnt = matrix_cnt + 1;
                    end
                else begin
                    addr_cam_auto = addr_cam_auto_tmp;
                    matrix_cnt = matrix_cnt_tmp;
                end
            end
        else begin
            addr_cam_auto = addr_cam_auto_tmp;
            matrix_cnt = matrix_cnt_tmp;
        end
    end
    
    /* state machine */
    always @(posedge clk or negedge rst_STATE or posedge int)
    begin
        if (!rst_STATE)
            begin
                st_cur <= START;
            end
        else if (int)
            begin
                st_cur <= STORE_TMP;
            end
        else
            begin
                st_cur <= st_next;
            end    
    end

    always @(posedge clk or negedge rst_STATE) 
    begin
        pass_tmp        <= pass;
        mask_C          <= 1;
        mask_F          <= 1;
        key_A_tmp       <= key_A;
        key_B_tmp       <= key_B;
        key_C_tmp       <= key_C;
        key_F_tmp       <= key_F;
        tmp_store_ddr_en <= store_ddr_en_reg;
        if (!rst_STATE)
            begin
                opt_cur         <= 0;
                mask            <= 0;
                bit_cnt         <= 0;
                rst_InC         <= 0;
                rst_InF         <= 0;
                tmp_bit_cnt     <= 0;
                tmp_pass        <= 0;
                tmp_mask        <= 0;
                tmp_C_F         <= 0;
                tmp_key_A       <= 0;
                tmp_key_B       <= 0;
                tmp_key_C       <= 0;
                tmp_key_F       <= 0;
                ret_addr        <= 0;
                ctxt_addr       <= 0;
                tag_C_F         <= 0;
                input_C         <= 0;
                input_F         <= 0;
                data_out_cbc_tmp<= 0;
                data_out_rbr_tmp<= 0;
                addr_cam_auto_tmp <= 0;
                matrix_cnt_tmp  <= 0;
                matrix_select_reg <= 0;
            end
        else begin
            case (st_cur)
                START:
                    begin
                        bit_cnt         <= 0;
                        opt_cur         <= op_code;
                        mask            <= 1;
                        tmp_bit_cnt     <= 0;
                        tmp_pass        <= 0;
                        tmp_mask        <= 0;
                        tmp_C_F         <= 0;
                        tmp_key_A       <= 0;
                        tmp_key_B       <= 0;
                        tmp_key_C       <= 0;
                        tmp_key_F       <= 0;
                        ret_addr        <= 0;
                        ctxt_addr       <= 0;
                        addr_cam_auto_tmp <= addr_cam_auto;
                        matrix_cnt_tmp  <= matrix_cnt;
                        matrix_select_reg <= matrix_select;
                        if ((op_code_valid == ADD) || (op_code_valid == SUB))
                            begin
                                rst_InC <= 1;
                            end
                        else if ((op_code_valid == ABS) || (op_code_valid == TSC))
                            begin
                                rst_InF <= 1;
                            end
                        else begin
                            rst_InF <= 0;
                            rst_InC <= 0;
                            input_C <= 0;
                            input_F <= 0;
                        end
                    end
                LOAD_TMP:
                    begin
                        /*if (ctxt_rdy == 1)
                            begin
                                bit_cnt <= tmp_bit_cnt_ret;
                                mask <= tmp_mask_ret;
                            end*/
                        if (ctxt_rdy == 1)
                        begin
                            bit_cnt     <= tmp_bit_cnt_ret;
                            mask        <= tmp_mask_ret;
                            if (tag_C_F == 1)
                                begin
                                    input_C <= tmp_C_F_ret;
                                end
                            else if (tag_C_F == 0)
                                begin
                                    input_F <= tmp_C_F_ret;
                                end
                            else begin
                                    input_C <= tmp_C_F_ret;
                                    input_F <= tmp_C_F_ret;
                            end
                        end
                    end
                LOAD_CTXT:
                    begin
                        if ((matrix_cnt == 0) && (op_code != RET))
                            begin
                                rst_InC <= 1;
                                rst_InF <= 1;
                            end
                    end
                LOAD_CTXT_FINISH_CHECK:
                    begin
                        addr_cam_auto_tmp <= addr_cam_auto;
                        matrix_cnt_tmp <= matrix_cnt;
                    end
                STORE_RBR:
                    begin
                        case (matrix_select_reg)
                        M_A:
                        begin
                            data_out_rbr_tmp <= data_A_rbr;
                        end

                        M_B:
                        begin
                            data_out_rbr_tmp <= data_B_rbr;
                        end

                        M_R:
                        begin
                            data_out_rbr_tmp <= data_R_rbr;
                        end

                        default:;
                        endcase
                    end
                STORE_CBC:
                    begin
                        case (matrix_select_reg)
                        M_A:
                        begin
                            data_out_cbc_tmp <= data_A_cbc;
                        end

                        M_B:
                        begin
                            data_out_cbc_tmp <= data_B_cbc;
                        end

                        M_R:
                        begin
                            data_out_cbc_tmp <= data_R_cbc;
                        end

                        default:;
                        endcase
                    end
                STORE_CTXT:
                    begin
                        case (matrix_cnt)
                            1: data_out_cbc_tmp <= data_A_cbc;
                            2: data_out_cbc_tmp <= data_B_cbc;
                            3: data_out_cbc_tmp <= data_R_cbc;
                            default:;
                        endcase
                    end
                STORE_TMP:
                    begin
                        tmp_bit_cnt     <= bit_cnt;
                        tmp_pass        <= pass;
                        tmp_mask        <= mask;
                        tmp_key_A       <= key_A;
                        tmp_key_B       <= key_B;
                        tmp_key_C       <= key_C;
                        tmp_key_F       <= key_F;
                        ret_addr        <= addr_cur_ins;
                        ctxt_addr       <= addr_cur_ctxt;
                        if(opt_cur == ADD || opt_cur == SUB)
                            begin
                                tmp_C_F <= data_C;
                                tag_C_F <= 1;
                            end
                        else if(opt_cur == TSC || opt_cur == ABS)
                            begin
                                tmp_C_F <= data_F;
                                tag_C_F <= 0;
                            end
                        else tmp_C_F <= 0;
                    end
                STORE_CTXT_FINISH_CHECK:
                    begin
                        addr_cam_auto_tmp <= addr_cam_auto;
                        matrix_cnt_tmp <= matrix_cnt;
                    end
                PASS_4_ADD:
                    begin
                        bit_cnt <= bit_cnt + 1;
                    end
                PASS_4_SUB:
                    begin
                        bit_cnt <= bit_cnt + 1;
                    end
                PASS_4_ABS:
                    begin
                        bit_cnt <= bit_cnt + 1;
                    end
                PASS_3_TSC:
                    begin
                        bit_cnt <= bit_cnt + 1;
                    end
                RET_STATE:
                    begin
                        opt_cur <= op_code;
                    end
                FINISH_CK:
                    begin
                        if (bit_cnt < DATA_WIDTH)
                            begin
                                mask <= mask << 1;
                            end
                    end 
                default:;
            endcase
        end
    end

    always @(op_code or addr_cam_auto)
    begin
        if (op_code == STORERBR 
            || op_code == STORECBC 
            || (addr_cam_auto == DATA_WIDTH - 1)&&(st_cur == STORE_CTXT_FINISH_CHECK))
        begin
            store_ddr_en_reg = 1;
        end
        else begin
            store_ddr_en_reg = 0;
        end
    end

    /* state transfer */
    always @ (*)
    begin
        case (st_cur)
            START:
                begin
                    pass                = 0;
                    key_A               = 0;
                    key_B               = 0;
                    key_C               = 0;
                    key_F               = 0;
                    rst_tag             = 0;
                    ABS_opt             = 0;
                    rst_InA             = 1;
                    rst_InB             = 1;
                    rst_InR             = 1;
                    addr_mem_col        = 0;
                    data_cmd            = 0;
                    ret_valid           = 0;
                    store_ctxt_finish   = 0;
                    ret_addr_pc_rdy     = 0;
                    int_set             = 0;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc         = 0;
                    input_B_cbc         = 0;
                    input_R_cbc         = 0;
                    addr_output_rbr_A   = DATA_DEPTH + 3;
                    addr_output_rbr_B   = DATA_DEPTH + 3;
                    addr_output_rbr_R   = DATA_DEPTH + 3;
                    addr_output_cbc_A   = DATA_WIDTH + 3;
                    addr_output_cbc_B   = DATA_WIDTH + 3;
                    addr_output_cbc_R   = DATA_WIDTH + 3;
                    data_out_cbc        = data_out_cbc_tmp;
                    data_out_rbr        = data_out_rbr_tmp;
                    //matrix_select_reg   = matrix_select;
                    case (op_code_valid)
                        RESET:
                            begin
                                ins_inp_valid   = 1;
                                st_next         = START;
                            end

                        RET:
                            begin
                                ins_inp_valid   = 1;
                                st_next         = LOAD_TMP;
                            end

                        LOADRBR:
                            begin
                                ins_inp_valid   = 0;
                                st_next         = LOAD_RBR;
                            end
                            
                        LOADCBC:
                            begin
                                ins_inp_valid   = 0;
                                st_next         = LOAD_CBC;
                            end

                        COPY:
                            begin
                                ins_inp_valid   = 1;
                                st_next         = COPY_MT;
                            end

                        STORERBR:
                            begin
                                ins_inp_valid   = 1;
                                st_next         = STORE_RBR;
                            end

                        STORECBC:
                            begin
                                ins_inp_valid   = 1;
                                st_next         = STORE_CBC;
                            end

                        ADD:
                            begin
                                ins_inp_valid   = 0;
                                st_next         = PASS_1_ADD;
                            end

                        SUB:
                            begin
                                ins_inp_valid   = 0;
                                st_next         = PASS_1_SUB;
                            end

                        ABS:
                            begin
                                ins_inp_valid   = 0;
                                st_next         = PASS_1_ABS;
                            end

                        TSC:
                            begin
                                ins_inp_valid   = 0;
                                st_next         = PASS_1_TSC;
                            end
                        default: 
                            begin
                                st_next        = START;
                                ins_inp_valid  = 1;
                            end
                    endcase
                end

            LOAD_RBR:
                begin
                    inout_mode      = RowxRow;
                    data_addr       = addr_mem;
                    data_cmd        = RowxRow_load;
                    pass            = 0;
                    key_A           = 0;
                    key_B           = 0;
                    key_C           = 0;
                    key_F           = 0;
                    rst_tag         = 0;
                    ABS_opt         = 0;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    input_A_cbc         = 0;
                    input_B_cbc         = 0;
                    input_R_cbc         = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    case (matrix_select_reg)
                        M_A: 
                        begin
                            rst_InA         = 0;
                            rst_InB         = 1;
                            rst_InR         = 1;
                            addr_input_rbr_A    = addr_cam;
                            addr_input_rbr_B    = 0;
                            addr_input_rbr_R    = 0;
                            if (data_cache_rdy)
                                begin
                                    input_A_rbr = data_in_rbr;
                                    input_B_rbr = 0;
                                    input_R_rbr = 0;
                                    st_next     = START;
                                    ins_inp_valid   = 1;
                                end
                            else
                                begin
                                    ins_inp_valid   = 0;
                                    input_A_rbr     = 0;
                                    input_B_rbr     = 0;
                                    input_R_rbr     = 0;
                                    st_next         = LOAD_RBR;
                                end
                        end

                        M_B: 
                        begin
                            rst_InA         = 1;
                            rst_InB         = 0;
                            rst_InR         = 1;
                            addr_input_rbr_B    = addr_cam;
                            addr_input_rbr_A    = 0;
                            addr_input_rbr_R    = 0;
                            if (data_cache_rdy)
                                begin
                                    input_B_rbr = data_in_rbr;
                                    input_A_rbr = 0;
                                    input_R_rbr = 0;
                                    st_next     = START;
                                    ins_inp_valid   = 1;
                                end
                            else
                                begin
                                    ins_inp_valid   = 0;
                                    input_A_rbr     = 0;
                                    input_B_rbr     = 0;
                                    input_R_rbr     = 0;
                                    st_next     = LOAD_RBR;
                                end
                        end

                        M_R: 
                        begin
                            rst_InA         = 1;
                            rst_InB         = 1;
                            rst_InR         = 0;
                            addr_input_rbr_R    = addr_cam;
                            addr_input_rbr_A    = 0;
                            addr_input_rbr_B    = 0;
                            if (data_cache_rdy)
                                begin
                                    input_R_rbr = data_in_rbr;
                                    input_A_rbr = 0;
                                    input_B_rbr = 0;
                                    st_next     = START;
                                    ins_inp_valid   = 1;
                                end
                            else
                                begin
                                    ins_inp_valid   = 0;
                                    input_A_rbr     = 0;
                                    input_B_rbr     = 0;
                                    input_R_rbr     = 0;
                                    st_next     = LOAD_RBR;
                                end
                        end

                        default: 
                            begin
                                ins_inp_valid   = 0;
                                st_next         = LOAD_RBR;
                                rst_InA         = 1;
                                rst_InB         = 1;
                                rst_InR         = 1;
                                addr_input_rbr_A    = 0;
                                addr_input_rbr_B    = 0;
                                addr_input_rbr_R    = 0;
                                input_A_rbr         = 0;
                                input_B_rbr         = 0;
                                input_R_rbr         = 0;
                            end
                            
                    endcase
                end

            LOAD_CBC:
                begin
                    inout_mode      = ColxCol;
                    data_addr       = addr_mem;
                    data_cmd        = ColxCol_load;
                    addr_cam_col    = addr_cam;
                    pass            = 0;
                    key_A           = 0;
                    key_B           = 0;
                    key_C           = 0;
                    key_F           = 0;
                    rst_tag         = 0;
                    ABS_opt         = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    case (matrix_select_reg)
                        M_A:
                        begin
                            rst_InA         = 0;
                            rst_InB         = 1;
                            rst_InR         = 1;
                            addr_input_cbc_A    = addr_cam;
                            addr_input_cbc_B    = 0;
                            addr_input_cbc_R    = 0;
                            if (data_cache_rdy)
                                begin
                                    input_A_cbc = data_in_cbc;
                                    input_B_cbc = 0;
                                    input_R_cbc = 0;
                                    if (ret_valid == 0)
                                        begin
                                            st_next     = START;
                                            ins_inp_valid   = 1;
                                        end
                                    else begin
                                        st_next = LOAD_CTXT;
                                        ins_inp_valid   = 0;
                                    end
                                end
                            else
                                begin
                                    st_next     = LOAD_CBC;
                                    ins_inp_valid   = 0;
                                    input_A_cbc = 0;
                                    input_B_cbc = 0;
                                    input_R_cbc = 0;
                                end
                        end

                        M_B:
                        begin
                            rst_InA         = 1;
                            rst_InB         = 0;
                            rst_InR         = 1;
                            addr_input_cbc_B    = addr_cam;
                            addr_input_cbc_A    = 0;
                            addr_input_cbc_R    = 0;
                            if (data_cache_rdy)
                                begin
                                    input_B_cbc = data_in_cbc;
                                    input_A_cbc = 0;
                                    input_R_cbc = 0;
                                    if (ret_valid == 0)
                                        begin
                                            st_next     = START;
                                            ins_inp_valid   = 1;
                                        end
                                    else begin
                                        st_next = LOAD_CTXT;
                                        ins_inp_valid   = 0;
                                    end
                                end
                            else
                                begin
                                    st_next     = LOAD_CBC;
                                    ins_inp_valid   = 0;
                                    input_A_cbc = 0;
                                    input_B_cbc = 0;
                                    input_R_cbc = 0;
                                end
                        end

                        M_R:
                        begin
                            rst_InA         = 1;
                            rst_InB         = 1;
                            rst_InR         = 0;
                            addr_input_cbc_R    = addr_cam;
                            addr_input_cbc_A    = 0;
                            addr_input_cbc_B    = 0;
                            if (data_cache_rdy)
                                begin
                                    input_R_cbc = data_in_cbc;
                                    input_A_cbc = 0;
                                    input_B_cbc = 0;
                                    if (ret_valid == 0)
                                        begin
                                            st_next     = START;
                                            ins_inp_valid   = 1;
                                        end
                                    else begin
                                        st_next = LOAD_CTXT;
                                        ins_inp_valid   = 0;
                                    end
                                end
                            else
                                begin
                                    st_next     = LOAD_CBC;
                                    ins_inp_valid   = 0;
                                    input_A_cbc = 0;
                                    input_B_cbc = 0;
                                    input_R_cbc = 0;
                                end
                        end

                        default:
                            begin    
                                st_next         = LOAD_CBC;
                                ins_inp_valid   = 0;
                                rst_InA         = 1;
                                rst_InB         = 1;
                                rst_InR         = 1;
                                addr_input_cbc_A    = 0;
                                addr_input_cbc_B    = 0;
                                addr_input_cbc_R    = 0;
                                input_A_cbc         = 0;
                                input_B_cbc         = 0;
                                input_R_cbc         = 0;
                            end
                    endcase
                end
            
            COPY_MT:
                begin
                    ins_inp_valid   = 0;
                    pass            = 0;
                    key_A           = 0;
                    key_B           = 0;
                    key_C           = 0;
                    key_F           = 0;
                    rst_tag         = 0;
                    ABS_opt         = 0;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc         = 0;
                    input_B_cbc         = 0;
                    input_R_cbc         = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    case (matrix_select_1)
                        M_A:
                        begin
                            rst_InA         = 0;
                            rst_InB         = 1;
                            rst_InR         = 1;
                            case (matrix_select_reg)
                                M_B:
                                begin
                                    inout_mode  = COPY_B;
                                    st_next     = START;
                                end
                                M_R:
                                begin
                                    inout_mode  = COPY_R;
                                    st_next     = START;
                                end
                                default: st_next= COPY_MT;
                            endcase
                        end

                        M_B:
                        begin
                            rst_InA         = 1;
                            rst_InB         = 0;
                            rst_InR         = 1;
                            case (matrix_select_reg)
                                M_A:
                                begin
                                    inout_mode  = COPY_A;
                                    st_next     = START;
                                end
                                M_R:
                                begin
                                    inout_mode  = COPY_R;
                                    st_next     = START;
                                end
                                default: st_next= COPY_MT;
                            endcase
                        end

                        M_R:
                        begin
                            rst_InA         = 1;
                            rst_InB         = 1;
                            rst_InR         = 0;
                            case (matrix_select_reg)
                                M_A:
                                begin
                                    inout_mode  = COPY_A;
                                    st_next     = START;
                                end
                                M_B:
                                begin
                                    inout_mode  = COPY_B;
                                    st_next     = START;
                                end
                                default: st_next= COPY_MT;
                            endcase
                        end

                        default: begin
                            st_next         = COPY_MT;
                            rst_InA         = 1;
                            rst_InB         = 1;
                            rst_InR         = 1;
                        end
                    endcase
                end

            STORE_RBR:
                begin
                    inout_mode      = RowxRow;
                    data_addr       = addr_mem;
                    data_cmd        = RowxRow_store;
                    ins_inp_valid   = 0;
                    pass            = 0;
                    key_A           = 0;
                    key_B           = 0;
                    key_C           = 0;
                    key_F           = 0;
                    rst_tag         = 0;
                    ABS_opt         = 0;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc         = 0;
                    input_B_cbc         = 0;
                    input_R_cbc         = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    case (matrix_select_reg)
                        M_B:
                        begin
                            addr_output_rbr_B   = addr_cam;
                            addr_output_rbr_A   = 0;
                            addr_output_rbr_R   = 0;
                            data_out_rbr        = data_B_rbr;
                            st_next             = START;
                        end

                        M_R:
                        begin
                            addr_output_rbr_R   = addr_cam;
                            addr_output_rbr_A   = 0;
                            addr_output_rbr_B   = 0;
                            data_out_rbr        = data_R_rbr;
                            st_next             = START;
                        end

                        M_A:
                        begin
                            addr_output_rbr_A   = addr_cam;
                            addr_output_rbr_B   = 0;
                            addr_output_rbr_R   = 0;
                            data_out_rbr        = data_A_rbr;
                            st_next             = START;
                        end

                        default: begin
                            st_next             = STORE_RBR;
                            addr_output_rbr_A   = DATA_DEPTH + 3;
                            addr_output_rbr_B   = DATA_DEPTH + 3;
                            addr_output_rbr_R   = DATA_DEPTH + 3;
                            data_out_rbr        = 0;
                        end
                    endcase
                end

            STORE_CBC:
                begin
                    inout_mode      = ColxCol;
                    data_addr       = addr_mem;
                    data_cmd        = ColxCol_store;
                    ins_inp_valid   = 0;
                    addr_cam_col    = addr_cam;
                    pass            = 0;
                    key_A           = 0;
                    key_B           = 0;
                    key_C           = 0;
                    key_F           = 0;
                    rst_tag         = 0;
                    ABS_opt         = 0;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc         = 0;
                    input_B_cbc         = 0;
                    input_R_cbc         = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    data_out_rbr        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    case (matrix_select_reg)
                        M_A:
                        begin
                            addr_output_cbc_A   = addr_cam_col;
                            addr_output_cbc_B   = 0;
                            addr_output_cbc_R   = 0;
                            data_out_cbc        = data_A_cbc;
                            st_next             = START;
                        end

                        M_B:
                        begin
                            addr_output_cbc_B   = addr_cam_col;
                            addr_output_cbc_A   = 0;
                            addr_output_cbc_R   = 0;
                            data_out_cbc        = data_B_cbc;
                            st_next             = START;
                        end

                        M_R:
                        begin
                            addr_output_cbc_R   = addr_cam_col;
                            addr_output_cbc_A   = 0;
                            addr_output_cbc_B   = 0;
                            data_out_cbc        = data_R_cbc;
                            st_next             = START;
                        end

                        default: begin
                            st_next             = STORE_CBC;
                            addr_output_cbc_A   = DATA_WIDTH + 3;
                            addr_output_cbc_B   = DATA_WIDTH + 3;
                            addr_output_cbc_R   = DATA_WIDTH + 3;
                            data_out_cbc        = 0;
                        end
                    endcase
                end

            STORE_TMP:
                begin
                    pass            = pass_tmp;
                    key_A           = key_A_tmp;
                    key_B           = key_B_tmp;
                    key_C           = key_C_tmp;
                    key_F           = key_F_tmp;
                    ins_inp_valid   = 0;
                    rst_tag         = 0;
                    ABS_opt         = 0;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc         = 0;
                    input_B_cbc         = 0;
                    input_R_cbc         = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    int_set             = 1;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    st_next             = STORE_CTXT;
                end

            STORE_CTXT:
                begin
                    inout_mode      = ColxCol;
                    data_cmd        = ColxCol_store;
                    ins_inp_valid   = 0;
                    addr_cam_col    = addr_cam_auto;
                    pass            = 0;
                    key_A           = 0;
                    key_B           = 0;
                    key_C           = 0;
                    key_F           = 0;
                    rst_tag         = 0;
                    ABS_opt         = 0;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc         = 0;
                    input_B_cbc         = 0;
                    input_R_cbc         = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    data_out_rbr        = 0;
                    ret_valid           = 0;
                    int_set             = 1;
                    if(matrix_cnt == 1)
                        begin
                            data_addr           = addr_cur_ctxt;
                            addr_output_cbc_A   = addr_cam_col;
                            addr_output_cbc_B   = 0;
                            addr_output_cbc_R   = 0;
                            data_out_cbc        = data_A_cbc;
                            st_next             = STORE_CTXT_FINISH_CHECK;
                        end
                    else if (matrix_cnt == 2)
                        begin
                            data_addr           = addr_cur_ctxt + DATA_DEPTH;
                            addr_output_cbc_B   = addr_cam_col;
                            addr_output_cbc_A   = 0;
                            addr_output_cbc_R   = 0;
                            data_out_cbc        = data_B_cbc;
                            st_next             = STORE_CTXT_FINISH_CHECK;
                        end
                    else if (matrix_cnt == 3)
                        begin
                            if (addr_cam_auto == DATA_WIDTH && data_cache_rdy == 1)
                                begin
                                    st_next             = GET_JMP_ADDR;
                                    addr_output_cbc_A   = 0;
                                    addr_output_cbc_B   = 0;
                                    addr_output_cbc_R   = 0;
                                    data_out_cbc        = 0;
                                end
                            else begin
                                data_addr           = addr_cur_ctxt + DATA_DEPTH + DATA_DEPTH;
                                addr_output_cbc_R   = addr_cam_col;
                                addr_output_cbc_A   = 0;
                                addr_output_cbc_B   = 0;
                                data_out_cbc        = data_R_cbc;
                                st_next             = STORE_CTXT_FINISH_CHECK;
                            end
                        end
                    else begin
                        st_next             = STORE_CTXT;
                        addr_output_cbc_A   = 0;
                        addr_output_cbc_B   = 0;
                        addr_output_cbc_R   = 0;
                        data_out_cbc        = 0;
                    end
                end

            STORE_CTXT_FINISH_CHECK:
                begin
                    ins_inp_valid   = 0;
                    pass            = 0;
                    key_A           = 0;
                    key_B           = 0;
                    key_C           = 0;
                    key_F           = 0;
                    rst_tag         = 0;
                    ABS_opt         = 0;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc         = 0;
                    input_B_cbc         = 0;
                    input_R_cbc         = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = data_out_cbc_tmp;
                    ret_valid           = 0;
                    int_set             = 1;
                    if(addr_cam_auto < DATA_WIDTH)
                        begin
                            st_next         = STORE_CTXT;
                        end
                    else if (addr_cam_auto == DATA_WIDTH)
                        begin
                            st_next         = STORE_CTXT;
                            if (matrix_cnt == 3)
                            begin
                                store_ctxt_finish = 1;
                            end
                        end
                    
                    else begin
                        st_next         = START;
                    end
                end

            GET_JMP_ADDR:
                begin
                    ins_inp_valid   = 0;
                    inout_mode      = RowxRow;
                    data_addr       = 16'he001;
                    data_cmd        = Addr_load;
                    pass            = 0;
                    key_A           = 0;
                    key_B           = 0;
                    key_C           = 0;
                    key_F           = 0;
                    rst_tag         = 0;
                    ABS_opt         = 0;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc         = 0;
                    input_B_cbc         = 0;
                    input_R_cbc         = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 1;
                    if(jmp_addr_rdy == 1)
                        begin
                            st_next = JMP_INS;
                        end
                end

            JMP_INS:
                begin
                    ins_inp_valid   = 0;
                    data_cmd        = 0;
                    jmp_addr_pc     = jmp_addr;
                    pass            = 0;
                    key_A           = 0;
                    key_B           = 0;
                    key_C           = 0;
                    key_F           = 0;
                    rst_tag         = 0;
                    ABS_opt         = 0;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc         = 0;
                    input_B_cbc         = 0;
                    input_R_cbc         = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 1;
                    st_next         = START;
                end

            LOAD_TMP:
                begin
                    ins_inp_valid   = 0;
                    ret_valid       = 1;
                    rst_tag         = 0;
                    ABS_opt         = 0;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc         = 0;
                    input_B_cbc         = 0;
                    input_R_cbc         = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    int_set             = 0;
                    if (ctxt_rdy == 1)
                        begin
                            pass        = tmp_pass_ret;
                            key_A       = tmp_key_A_ret;
                            key_B       = tmp_key_B_ret;
                            key_C       = tmp_key_C_ret;
                            key_F       = tmp_key_F_ret;
                        end
                    else if (data_cache_rdy == 1)
                        begin
                            st_next     = LOAD_CTXT;
                            pass = pass_tmp;
                            key_A       = key_A_tmp;
                            key_B       = key_B_tmp;
                            key_C       = key_C_tmp;
                            key_F       = key_F_tmp;
                        end
                    else begin
                        st_next = LOAD_TMP;
                        pass = pass_tmp;
                        key_A       = key_A_tmp;
                        key_B       = key_B_tmp;
                        key_C       = key_C_tmp;
                        key_F       = key_F_tmp;
                        end
                end

            LOAD_CTXT:
                begin
                    inout_mode      = ColxCol;
                    data_cmd        = ColxCol_load;
                    ins_inp_valid   = 0;
                    addr_cam_col    = addr_cam_auto;
                    pass            = pass_tmp;
                    key_A           = key_A_tmp;
                    key_B           = key_B_tmp;
                    key_C           = key_C_tmp;
                    key_F           = key_F_tmp;
                    rst_tag         = 0;
                    ABS_opt         = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 1;
                    int_set             = 0;
                    if(matrix_cnt == 1)
                        begin
                            rst_InA             = 0;
                            rst_InB             = 1;
                            rst_InR             = 1;
                            addr_input_cbc_A    = addr_cam_col;
                            addr_input_cbc_B    = 0;
                            addr_input_cbc_R    = 0;
                            data_addr           = ctxt_addr_ret;
                            if (data_cache_rdy)
                                begin
                                    input_A_cbc = data_in_cbc;
                                    input_B_cbc = 0;
                                    input_R_cbc = 0;
                                    st_next     = LOAD_CTXT_FINISH_CHECK;
                                end
                            else begin
                                st_next         = LOAD_CTXT;
                                input_A_cbc     = 0;
                                input_B_cbc     = 0;
                                input_R_cbc     = 0;
                            end
                        end
                    else if (matrix_cnt == 2)
                        begin
                            rst_InA             = 1;
                            rst_InB             = 0;
                            rst_InR             = 1;
                            addr_input_cbc_B    = addr_cam_col;
                            addr_input_cbc_A    = 0;
                            addr_input_cbc_R    = 0;
                            data_addr           = ctxt_addr_ret + DATA_DEPTH;
                            if (data_cache_rdy)
                                begin
                                    input_B_cbc = data_in_cbc;
                                    input_A_cbc = 0;
                                    input_R_cbc = 0;
                                    st_next     = LOAD_CTXT_FINISH_CHECK;
                                end
                            else begin
                                st_next         = LOAD_CTXT;
                                input_A_cbc     = 0;
                                input_B_cbc     = 0;
                                input_R_cbc     = 0;
                            end
                        end
                    else if (matrix_cnt == 3)
                        begin
                            rst_InA             = 1;
                            rst_InB             = 1;
                            rst_InR             = 0;
                            addr_input_cbc_R    = addr_cam_col;
                            addr_input_cbc_A    = 0;
                            addr_input_cbc_B    = 0;
                            data_addr           = ctxt_addr_ret + DATA_DEPTH + DATA_DEPTH;
                            if (data_cache_rdy)
                                begin
                                    input_R_cbc = data_in_cbc;
                                    input_A_cbc     = 0;
                                    input_B_cbc     = 0;
                                    st_next     = LOAD_CTXT_FINISH_CHECK;
                                end
                            else begin
                                st_next         = LOAD_CTXT;
                                input_A_cbc     = 0;
                                input_B_cbc     = 0;
                                input_R_cbc     = 0;
                            end
                        end
                    else if (matrix_cnt == 0)
                        begin
                            ret_addr_pc     = ret_addr_ret;
                            ret_addr_pc_rdy = 1;
                            data_cmd        = 0;
                            rst_InA         = 1;
                            rst_InB         = 1;
                            rst_InR         = 1;
                            addr_input_cbc_A    = 0;
                            addr_input_cbc_B    = 0;
                            addr_input_cbc_R    = 0;
                            input_A_cbc     = 0;
                            input_B_cbc     = 0;
                            input_R_cbc     = 0;
                            if (op_code == RET)
                                begin
                                    st_next = LOAD_CTXT;
                                end
                            else begin
                                st_next     = RET_STATE;
                               // ret_valid   = 0;
                            end
                        end
                    else begin
                        st_next         = LOAD_CTXT;
                        rst_InA         = 1;
                        rst_InB         = 1;
                        rst_InR         = 1;
                        addr_input_cbc_A    = 0;
                        addr_input_cbc_B    = 0;
                        addr_input_cbc_R    = 0;
                        input_A_cbc     = 0;
                        input_B_cbc     = 0;
                        input_R_cbc     = 0;
                    end
                end

            LOAD_CTXT_FINISH_CHECK:
                begin
                    ins_inp_valid   = 0;
                    pass            = pass_tmp;
                    key_A           = key_A_tmp;
                    key_B           = key_B_tmp;
                    key_C           = key_C_tmp;
                    key_F           = key_F_tmp;
                    rst_tag         = 0;
                    ABS_opt         = 0;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc         = 0;
                    input_B_cbc         = 0;
                    input_R_cbc         = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 1;
                    int_set             = 0;
                    if(addr_cam_auto < DATA_WIDTH)
                        begin
                            st_next         = LOAD_CTXT;
                        end
                    else if (addr_cam_auto == DATA_WIDTH)
                        begin
                            st_next         = LOAD_CTXT;
                        end
                    
                    else begin
                        st_next         = START;
                    end
                end
            
            RET_STATE:
                begin
                    ins_inp_valid   = 0;
                    pass            = pass_tmp;
                    key_A           = key_A_tmp;
                    key_B           = key_B_tmp;
                    key_C           = key_C_tmp;
                    key_F           = key_F_tmp;
                    rst_tag         = 0;
                    ABS_opt         = 0;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc         = 0;
                    input_B_cbc         = 0;
                    input_R_cbc         = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    case(op_code)
                        ADD:
                            begin
                                st_next = RSTTAG_ADD;
                            end
                        SUB:
                            begin
                                st_next = RSTTAG_SUB;
                            end
                        TSC:
                            begin
                                st_next = RSTTAG_TSC;
                            end
                        ABS:
                            begin
                                st_next = RSTTAG_ABS;
                            end
                        default: st_next = RET_STATE;
                    endcase
                end

            /* pass of ADD */
            PASS_1_ADD:
                begin
                    ins_inp_valid   = 0;
                    rst_tag         = 1;
                    pass            = 1;
                    key_A           = 1;
                    key_B           = 1;
                    key_C           = 0;
                    key_F           = 0;
                    ABS_opt         = 0;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc         = 0;
                    input_B_cbc         = 0;
                    input_R_cbc         = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    st_next         = RSTTAG_ADD;
                end
                
            PASS_2_ADD:
                begin
                    ins_inp_valid   = 0;
                    rst_tag         = 1;
                    pass            = 2;
                    key_A           = 1;
                    key_B           = 0;
                    key_C           = 0;
                    key_F           = 0;
                    ABS_opt         = 0;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc         = 0;
                    input_B_cbc         = 0;
                    input_R_cbc         = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    st_next         = RSTTAG_ADD;
                end
                
            PASS_3_ADD:
                begin
                    ins_inp_valid   = 0;
                    rst_tag         = 1;
                    pass            = 3;
                    key_A           = 0;
                    key_B           = 0;
                    key_C           = 1;
                    key_F           = 0;
                    ABS_opt         = 0;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc         = 0;
                    input_B_cbc         = 0;
                    input_R_cbc         = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    st_next         = RSTTAG_ADD;
                end
            
            PASS_4_ADD:
                begin
                    ins_inp_valid   = 0;
                    rst_tag         = 1;
                    pass            = 4;
                    key_A           = 0;
                    key_B           = 1;
                    key_C           = 1;
                    key_F           = 0;
                    ABS_opt         = 0;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc         = 0;
                    input_B_cbc         = 0;
                    input_R_cbc         = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    st_next         = RSTTAG_ADD;
                end
                
            RSTTAG_ADD:
                begin
                    ins_inp_valid   = 0;
                    rst_tag         = 0;
                    pass            = pass_tmp;
                    key_A           = 0;
                    key_B           = 0;
                    key_C           = 0;
                    key_F           = 0;
                    ABS_opt         = 0;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc     = 0;
                    input_B_cbc     = 0;
                    input_R_cbc     = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    case(pass)
                    1: st_next      = PASS_2_ADD;
                    2: st_next      = PASS_3_ADD;
                    3: st_next      = PASS_4_ADD;
                    4: st_next      = FINISH_CK;
                    default: st_next = RSTTAG_ADD;
                    endcase
                end       

            /* pass of SUB */
            PASS_1_SUB:
                begin
                    ins_inp_valid   = 0;
                    rst_tag         = 1;
                    pass            = 1;
                    key_A           = 1;
                    key_B           = 0;
                    key_C           = 0;
                    key_F           = 0;
                    ABS_opt         = 0;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc     = 0;
                    input_B_cbc     = 0;
                    input_R_cbc     = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    st_next = RSTTAG_SUB;
                end
                
            PASS_2_SUB:
                begin
                    ins_inp_valid   = 0;
                    rst_tag         = 1;
                    pass            = 2;
                    key_A           = 1;
                    key_B           = 1;
                    key_C           = 0;
                    key_F           = 0;
                    ABS_opt         = 0;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc     = 0;
                    input_B_cbc     = 0;
                    input_R_cbc     = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    st_next         = RSTTAG_SUB;
                end
                
            PASS_3_SUB:
                begin
                    ins_inp_valid   = 0;
                    rst_tag         = 1;
                    pass            = 3;
                    key_A           = 0;
                    key_B           = 1;
                    key_C           = 1;
                    key_F           = 0;
                    ABS_opt         = 0;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc     = 0;
                    input_B_cbc     = 0;
                    input_R_cbc     = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    st_next         = RSTTAG_SUB;
                end
            
            PASS_4_SUB:
                begin
                    ins_inp_valid   = 0;
                    rst_tag         = 1;
                    pass            = 4;
                    key_A           = 0;
                    key_B           = 0;
                    key_C           = 1;
                    key_F           = 0;
                    ABS_opt         = 0;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc     = 0;
                    input_B_cbc     = 0;
                    input_R_cbc     = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    st_next         = RSTTAG_SUB;
                end
                
            RSTTAG_SUB:
                begin
                    rst_tag         = 0;
                    ins_inp_valid   = 0;
                    pass            = pass_tmp;
                    key_A           = 0;
                    key_B           = 0;
                    key_C           = 0;
                    key_F           = 0;
                    ABS_opt         = 0;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc     = 0;
                    input_B_cbc     = 0;
                    input_R_cbc     = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    case(pass)
                    1: st_next      = PASS_2_SUB;
                    2: st_next      = PASS_3_SUB;
                    3: st_next      = PASS_4_SUB;
                    4: st_next      = FINISH_CK;
                    default: st_next = RSTTAG_SUB;
                    endcase
                end

            /* pass of ABS */          
            PASS_1_ABS:
                begin
                    ins_inp_valid   = 0;
                    rst_tag         = 1;
                    pass            = 1;
                    key_A           = 1;
                    key_B           = 0;
                    key_C           = 0;
                    key_F           = 0;
                    ABS_opt         = 1;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc     = 0;
                    input_B_cbc     = 0;
                    input_R_cbc     = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    st_next         = RSTTAG_ABS;
                end
                
            PASS_2_ABS:
                begin
                    ins_inp_valid   = 0;
                    rst_tag         = 1;
                    pass            = 2;
                    key_A           = 0;
                    key_B           = 0;
                    key_C           = 0;
                    key_F           = 1;
                    ABS_opt         = 1;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc     = 0;
                    input_B_cbc     = 0;
                    input_R_cbc     = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    st_next         = RSTTAG_ABS;
                end
                
            PASS_3_ABS:
                begin
                    ins_inp_valid   = 0;
                    rst_tag         = 1;
                    pass            = 3;
                    key_A           = 1;
                    key_B           = 0;
                    key_C           = 0;
                    key_F           = 1;
                    ABS_opt         = 1;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc     = 0;
                    input_B_cbc     = 0;
                    input_R_cbc     = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    st_next         = RSTTAG_ABS;
                end
            
            PASS_4_ABS:
                begin
                    ins_inp_valid   = 0;
                    rst_tag         = 1;
                    pass            = 4;
                    key_A           = 1;
                    key_B           = 0;
                    key_C           = 0;
                    key_F           = 0;
                    ABS_opt         = 1;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc     = 0;
                    input_B_cbc     = 0;
                    input_R_cbc     = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    st_next         = RSTTAG_ABS;
                end
                
            RSTTAG_ABS:
                begin
                    rst_tag         = 0; 
                    ins_inp_valid   = 0;
                    pass            = pass_tmp;
                    key_A           = 0;
                    key_B           = 0;
                    key_C           = 0;
                    key_F           = 0;
                    ABS_opt         = 1;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc     = 0;
                    input_B_cbc     = 0;
                    input_R_cbc     = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    case(pass)
                    1: st_next      = PASS_2_ABS;
                    2: st_next      = PASS_3_ABS;
                    3: st_next      = PASS_4_ABS;
                    4: st_next      = FINISH_CK;
                    default: st_next = RSTTAG_ABS;
                    endcase
                end

            /* pass of TSC */        
            PASS_1_TSC:
                begin
                    ins_inp_valid   = 0;
                    rst_tag         = 1;
                    pass            = 1;
                    key_A           = 0;
                    key_B           = 0;
                    key_C           = 0;
                    key_F           = 1;
                    ABS_opt         = 0;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc     = 0;
                    input_B_cbc     = 0;
                    input_R_cbc     = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    st_next         = RSTTAG_TSC;
                end
                
            PASS_2_TSC:
                begin
                    ins_inp_valid   = 0;
                    rst_tag         = 1;
                    pass            = 2;
                    key_A           = 1;
                    key_B           = 0;
                    key_C           = 0;
                    key_F           = 1;
                    ABS_opt         = 0;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc     = 0;
                    input_B_cbc     = 0;
                    input_R_cbc     = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    st_next         = RSTTAG_TSC;
                end
                
            PASS_3_TSC:
                begin
                    ins_inp_valid   = 0;
                    rst_tag         = 1;
                    pass            = 3;
                    key_A           = 1;
                    key_B           = 0;
                    key_C           = 0;
                    key_F           = 0;
                    ABS_opt         = 0;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc     = 0;
                    input_B_cbc     = 0;
                    input_R_cbc     = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    st_next         = RSTTAG_TSC;
                end
                
            RSTTAG_TSC:
                begin
                    rst_tag         = 0; 
                    ins_inp_valid   = 0; 
                    pass            = pass_tmp;
                    key_A           = 0;
                    key_B           = 0;
                    key_C           = 0;
                    key_F           = 0;
                    ABS_opt         = 1;
                    rst_InA         = 1;
                    rst_InB         = 1;
                    rst_InR         = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc     = 0;
                    input_B_cbc     = 0;
                    input_R_cbc     = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    case(pass)
                    1: st_next = PASS_2_TSC;
                    2: st_next = PASS_3_TSC;
                    3: st_next = FINISH_CK;
                    default: st_next = RSTTAG_TSC;
                    endcase
                end
    
            FINISH_CK:
                begin
                    pass    = pass_tmp;
                    key_A   = key_A_tmp;
                    key_B   = key_B_tmp;
                    key_C   = key_C_tmp;
                    key_F   = key_F_tmp;
                    rst_tag = 0;
                    ABS_opt = 0;
                    rst_InA = 1;
                    rst_InB = 1;
                    rst_InR = 1;
                    addr_input_cbc_A    = 0;
                    addr_input_cbc_B    = 0;
                    addr_input_cbc_R    = 0;
                    addr_input_rbr_A    = 0;
                    addr_input_rbr_B    = 0;
                    addr_input_rbr_R    = 0;
                    input_A_rbr         = 0;
                    input_B_rbr         = 0;
                    input_R_rbr         = 0;
                    input_A_cbc     = 0;
                    input_B_cbc     = 0;
                    input_R_cbc     = 0;
                    addr_output_rbr_A   = 0;
                    addr_output_rbr_B   = 0;
                    addr_output_rbr_R   = 0;
                    addr_output_cbc_A   = 0;
                    addr_output_cbc_B   = 0;
                    addr_output_cbc_R   = 0;
                    data_out_rbr        = 0;
                    data_out_cbc        = 0;
                    ret_valid           = 0;
                    int_set             = 0;
                    if(bit_cnt < DATA_WIDTH)
                        begin
                            ins_inp_valid   = 0;
                            case(opt_cur)
                                ADD: st_next = PASS_1_ADD;
                                SUB: st_next = PASS_1_SUB;
                                TSC: st_next = PASS_1_TSC;
                                ABS: st_next = PASS_1_ABS;
                                default: st_next = FINISH_CK;
                            endcase
                        end
                    else begin
                        st_next         = START;
                        ins_inp_valid   = 1;
                        end
                end
            
            default: begin
                st_next         = START;
                ins_inp_valid   = 0;
                pass            = pass_tmp;
                key_A           = key_A_tmp;
                key_B           = key_B_tmp;
                key_C           = key_C_tmp;
                key_F           = key_F_tmp;
                rst_tag         = 0;
                ABS_opt         = 1;
                rst_InA         = 1;
                rst_InB         = 1;
                rst_InR         = 1;
                addr_input_cbc_A    = 0;
                addr_input_cbc_B    = 0;
                addr_input_cbc_R    = 0;
                addr_input_rbr_A    = 0;
                addr_input_rbr_B    = 0;
                addr_input_rbr_R    = 0;
                input_A_rbr         = 0;
                input_B_rbr         = 0;
                input_R_rbr         = 0;
                input_A_cbc     = 0;
                input_B_cbc     = 0;
                input_R_cbc     = 0;
                addr_output_rbr_A   = 0;
                addr_output_rbr_B   = 0;
                addr_output_rbr_R   = 0;
                addr_output_cbc_A   = 0;
                addr_output_cbc_B   = 0;
                addr_output_cbc_R   = 0;
                data_out_rbr        = 0;
                data_out_cbc        = 0;
                ret_valid           = 0;
                int_set             = 0;
                end

        endcase
    end

endmodule