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
    input wire                              rst,
    input wire                              int,
    output reg [DATA_WIDTH - 1 : 0]         data_print,
    output reg                              data_print_rdy,
    output wire                             finish_flag,

    /* the interface of instruction cache */
    input wire [OPCODE_WIDTH - 1 : 0]       ins_valid,
    input wire [ISA_WIDTH - 1 : 0]          ins_to_apctrl,

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

    /* the interface of Program counter */
    input wire [ADDR_WIDTH_MEM - 1 : 0]     addr_ins,
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
    output reg                              abs_opt,
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
    output wire [DATA_WIDTH - 1 : 0]        mask_A,
    output reg [DATA_WIDTH - 1 : 0]         mask_B,
    output reg [DATA_WIDTH - 1 : 0]         mask_R,
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
    localparam                              PRINT           = 4'd13;
    localparam                              STOP            = 4'd14;

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
    localparam                              LOAD_CTXT_FC    = 6'd29;
    localparam                              STORE_TMP       = 6'd30;
    localparam                              STORE_CTXT      = 6'd31;
    localparam                              STORE_CTXT_FC   = 6'd32;
    localparam                              GET_JMP_ADDR    = 6'd33;
    localparam                              JMP_INS         = 6'd34;
    localparam                              RET_STATE       = 6'd35;
    localparam                              PRINT_DATA      = 6'd36;
    localparam                              STORE_END       = 6'd37;
    localparam                              STORE_END_2       = 6'd38;
    localparam                              FINISH          = 6'd39;

    /* inout_mode */
    localparam                              RowxRow         = 3'd1;
    localparam                              ColxCol         = 3'd2;
    localparam                              COPY_B          = 3'd3;
    localparam                              COPY_R          = 3'd4;
    localparam                              COPY_A          = 3'd5;
    localparam                              RST0            = 3'd6;

    /* data_cache command */
    localparam                              RowxRow_load    = 3'd1;
    localparam                              RowxRow_store   = 3'd2;
    localparam                              ColxCol_load    = 3'd3;
    localparam                              ColxCol_store   = 3'd4;
    localparam                              Addr_load       = 3'd5;

    /* state variables */
    
    reg [5 : 0]                             st_next;
    reg [5 : 0]                             st_cur;
    reg [OPCODE_WIDTH - 1 : 0]              opt_cur;
    reg [DATA_WIDTH - 1 : 0]                bit_cnt;

    reg [2 : 0]                             pass_tmp;
    reg                                     key_A_tmp;
    reg                                     key_B_tmp;
    reg                                     key_C_tmp;
    reg                                     key_F_tmp;
    reg [ADDR_WIDTH_MEM - 1 : 0]            data_addr_tmp;
    reg [ADDR_WIDTH_CAM - 1 : 0]            addr_cam_tmp;

    /* op_code and operands */
    wire [OPCODE_WIDTH - 1 : 0]             op_code;
    wire [OPCODE_WIDTH - 1 : 0]             op_code_valid;
    wire [ADDR_WIDTH_CAM - 1 : 0]           addr_cam;
    wire [OPRAND_2_WIDTH - 1 : 0]           matrix_select;
    reg  [OPRAND_2_WIDTH - 1 : 0]           matrix_select_reg;
    wire [OPRAND_2_WIDTH - 1 : 0]           matrix_select_1;
    wire [ADDR_WIDTH_MEM - 1 : 0]           addr_mem;
    reg [DATA_WIDTH - 1 : 0]                mask_reg;
    
    reg [ADDR_WIDTH_MEM - 1 : 0]            addr_mem_col;
    reg [1 : 0]                             matrix_cnt;

    reg                                     store_ddr_delay;
    wire                                    store_ddr;
    reg [ADDR_WIDTH_CAM - 1 : 0]            addr_cam_auto;
    reg                                     tag_C_F; /* indicate which one do we store when interrupt*/
                                                     /* 1 means C, 0 means F */
    reg [DATA_WIDTH - 1 : 0]                data_out_rbr_tmp;
    reg [DATA_DEPTH - 1 : 0]                data_out_cbc_tmp;
    reg [3 : 0]                             cam_clk_cnt;
    
    reg store_ddr_en_delay;
    wire store_ddr_en_1;
    assign store_ddr_en_1 = store_ddr_delay & ~store_ddr;
    assign store_ddr_en = store_ddr_en_1 | store_ddr_en_delay;
    assign finish_flag = (st_cur == FINISH)? 1 : 0;

    assign op_code = ins_to_apctrl [OPCODE_WIDTH + ADDR_WIDTH_CAM
                                + OPRAND_2_WIDTH + ADDR_WIDTH_MEM - 1 :
                                  ADDR_WIDTH_CAM + OPRAND_2_WIDTH 
                                + ADDR_WIDTH_MEM];

    assign addr_cam = ins_to_apctrl [ADDR_WIDTH_CAM + OPRAND_2_WIDTH 
                                + ADDR_WIDTH_MEM - 1 :
                                  OPRAND_2_WIDTH 
                                + ADDR_WIDTH_MEM];

    assign matrix_select = ins_to_apctrl [OPRAND_2_WIDTH + ADDR_WIDTH_MEM - 1 : ADDR_WIDTH_MEM];
    assign matrix_select_1 = addr_cam [OPRAND_2_WIDTH - 1 : 0];
    assign addr_mem = ins_to_apctrl [ADDR_WIDTH_MEM - 1 : 0];
    assign op_code_valid = op_code & ins_valid;     

    /* ALU */
    wire [DATA_WIDTH - 1 : 0]               arith_6;
    wire [1 : 0]                            arith_3;
    wire [ADDR_WIDTH_CAM - 1 : 0]           arith_2;
    wire [3 : 0]                            arith_1;
    wire [ADDR_WIDTH_MEM - 1 : 0]           arith_5;
    wire [ADDR_WIDTH_MEM - 1 : 0]           arith_4;
    wire [ADDR_WIDTH_MEM - 1 : 0]           arith_7;
    wire [ADDR_WIDTH_MEM - 1 : 0]           arith_8;
    wire [DATA_WIDTH - 1 : 0]               arith_9;

    wire                                    ctrl_exp_1;
    wire                                    ctrl_exp_2;
    wire                                    ctrl_exp_3;
    wire                                    ctrl_exp_4;
    wire                                    ctrl_exp_5;
    wire                                    ctrl_exp_6;
    wire                                    ctrl_exp_7;
    wire                                    ctrl_exp_8;
    wire                                    ctrl_exp_9;
    wire                                    ctrl_exp_10;
    wire                                    ctrl_exp_11;
    wire                                    ctrl_exp_12;

    wire                                    ctrl_exp_13;
    wire                                    ctrl_exp_14;
    wire                                    ctrl_exp_15;
    wire                                    ctrl_exp_16;

    localparam  DATA_DEPTH_P_3 = DATA_DEPTH + 3;
    localparam  DATA_WIDTH_P_3 = DATA_WIDTH + 3;
    assign      arith_1 = cam_clk_cnt + 1;
    assign      arith_2 = addr_cam_auto + 1;
    assign      arith_3 = matrix_cnt + 1;
    assign      arith_4 = ctxt_addr_ret + DATA_DEPTH + DATA_DEPTH;
    assign      arith_5 = ctxt_addr_ret + DATA_DEPTH;
    assign      arith_6 = bit_cnt + 1;
    assign      arith_7 = addr_cur_ctxt + DATA_DEPTH;
    assign      arith_8 = addr_cur_ctxt + DATA_DEPTH + DATA_DEPTH;
    assign      arith_9 = mask_reg << 1;

    assign      ctrl_exp_1 = (cam_clk_cnt == 0)? 1 : 0;
    assign      ctrl_exp_2 = (cam_clk_cnt == 1)? 1 : 0;
    assign      ctrl_exp_3 = (cam_clk_cnt == 7)? 1 : 0;
    assign      ctrl_exp_4 = (matrix_cnt == 0)? 1 : 0;
    assign      ctrl_exp_5 = (matrix_cnt == 1)? 1 : 0;
    assign      ctrl_exp_6 = (matrix_cnt == 2)? 1 : 0;
    assign      ctrl_exp_7 = (matrix_cnt == 3)? 1 : 0;
    assign      ctrl_exp_8 = (op_code == RET)? 1 : 0;
    assign      ctrl_exp_9 = (bit_cnt < DATA_WIDTH)? 1 : 0;
    assign      ctrl_exp_10 = (ctrl_exp_4 && ctrl_exp_8)? 1 : 0;
    assign      ctrl_exp_11 = (ctrl_exp_4 && !ctrl_exp_8)? 1 : 0;
    assign      ctrl_exp_12 = (op_code == STORERBR 
                      || op_code == STORECBC 
                      || (addr_cam_auto == DATA_WIDTH - 1)&&(st_cur == STORE_CTXT_FC))? 1 : 0;
    
    assign      ctrl_exp_13 = (addr_cam_auto < DATA_WIDTH)? 1 : 0;
    assign      ctrl_exp_14 = (addr_cam_auto <= DATA_WIDTH)? 1 : 0;
    assign      ctrl_exp_15 = (addr_cam_auto < DATA_WIDTH - 1)? 1 : 0;
    assign      ctrl_exp_16 = ((addr_cam_auto == DATA_WIDTH) && (matrix_cnt == 0))? 1 : 0;
    assign      mask_A = mask_reg;
    assign      store_ddr = (ctrl_exp_12)? 1 : 0;

    always @(*) begin
        case(op_code)
            ADD: mask_B = mask_reg;
            SUB: mask_B = mask_reg;
            default: mask_B = 0;
        endcase
    end

    always @(*) begin
        case(op_code)
            ABS: mask_R = mask_reg;
            TSC: mask_R = mask_reg;
            default: mask_R = 0;
        endcase
    end
    
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            cam_clk_cnt <= 0;
        end
        else begin
            case({st_cur, cam_clk_cnt[3]}) // cam_clk_cnt[3] == 0 means cam_clk_cnt < 7
                {PASS_1_ADD, 1'b0}: begin
                    cam_clk_cnt <= arith_1;
                end
                {PASS_1_SUB, 1'b0}: begin
                    cam_clk_cnt <= arith_1;
                end
                {PASS_1_ABS, 1'b0}: begin
                    cam_clk_cnt <= arith_1;
                end
                {PASS_1_TSC, 1'b0}: begin
                    cam_clk_cnt <= arith_1;
                end
                {PASS_2_ADD, 1'b0}: begin
                    cam_clk_cnt <= arith_1;
                end
                {PASS_2_SUB, 1'b0}: begin
                    cam_clk_cnt <= arith_1;
                end
                {PASS_2_ABS, 1'b0}: begin
                    cam_clk_cnt <= arith_1;
                end
                {PASS_2_TSC, 1'b0}: begin
                    cam_clk_cnt <= arith_1;
                end
                {PASS_3_ADD, 1'b0}: begin
                    cam_clk_cnt <= arith_1;
                end
                {PASS_3_SUB, 1'b0}: begin
                    cam_clk_cnt <= arith_1;
                end
                {PASS_3_ABS, 1'b0}: begin
                    cam_clk_cnt <= arith_1;
                end
                {PASS_3_TSC, 1'b0}: begin
                    cam_clk_cnt <= arith_1;
                end
                {PASS_4_ADD, 1'b0}: begin
                    cam_clk_cnt <= arith_1;
                end
                {PASS_4_SUB, 1'b0}: begin
                    cam_clk_cnt <= arith_1;
                end
                {PASS_4_ABS, 1'b0}: begin
                    cam_clk_cnt <= arith_1;
                end
                {STORE_RBR, 1'b0}: begin
                    cam_clk_cnt <= arith_1;
                end
                {STORE_CBC, 1'b0}: begin
                    cam_clk_cnt <= arith_1;
                end
                {STORE_CTXT, 1'b0}: begin
                    cam_clk_cnt <= arith_1;
                end
                {STORE_END, 1'b0}: begin
                    case (ctrl_exp_1)
                        1'b1: cam_clk_cnt <= arith_1;
                        1'b0: cam_clk_cnt <= 0;
                        default:;
                    endcase
                end
                default: begin
                    cam_clk_cnt <= 0;
                end
            endcase
        end
    end

    always @(posedge clk) begin
        case (st_cur)
            START: begin
                addr_cam_auto <= 0;
                matrix_cnt <= 1;
            end
            STORE_CTXT_FC: begin
                case (ctrl_exp_13)
                    1: addr_cam_auto <= arith_2;
                    default:begin
                        addr_cam_auto <= 0;
                        matrix_cnt <= arith_3;
                    end
                endcase
            end
            LOAD_CTXT_FC: begin
                case (ctrl_exp_15)
                    1: addr_cam_auto <= arith_2;
                    default:begin
                        addr_cam_auto <= 0;
                        matrix_cnt <= arith_3;
                    end
                endcase
            end
            default:;
        endcase
    end 
    
    /* state machine */
    always @(posedge clk or negedge rst or posedge int) begin
        if (!rst) begin
            st_cur <= START;
        end
        else if (int) begin
            st_cur <= STORE_TMP;
        end
        else begin
           st_cur <= st_next;
        end    
    end

    always @(posedge clk or negedge rst) begin
        pass_tmp <= pass;
        key_A_tmp <= key_A;
        key_B_tmp <= key_B;
        key_C_tmp <= key_C;
        key_F_tmp <= key_F;
        store_ddr_delay <= store_ddr;
        store_ddr_en_delay <= store_ddr_en_1;
        addr_cam_tmp <= addr_cam;
        if (!rst) begin
            opt_cur <= 0;
            mask_reg <= 0;
            bit_cnt <= 0;
            rst_InC <= 0;
            rst_InF <= 0;
            tmp_bit_cnt <= 0;
            tmp_pass <= 0;
            tmp_mask <= 0;
            tmp_C_F <= 0;
            tmp_key_A <= 0;
            tmp_key_B <= 0;
            tmp_key_C <= 0;
            tmp_key_F <= 0;
            ret_addr <= 0;
            ctxt_addr <= 0;
            tag_C_F <= 0;
            input_C <= 0;
            input_F <= 0;
            data_out_cbc_tmp <= 0;
            data_out_rbr_tmp <= 0;
            matrix_select_reg <= 0;
            ret_addr_pc <= 0;
            ret_addr_pc_rdy <= 0;
            jmp_addr_pc <= 0;
            store_ctxt_finish <= 0;
            data_addr_tmp <= 0;
        end
        else begin
            case (st_cur)
                START: begin
                    bit_cnt <= 0;
                    opt_cur <= op_code;
                    mask_reg <= 1;
                    tmp_bit_cnt <= 0;
                    tmp_pass <= 0;
                    tmp_mask <= 0;
                    tmp_C_F <= 0;
                    tmp_key_A <= 0;
                    tmp_key_B <= 0;
                    tmp_key_C <= 0;
                    tmp_key_F <= 0;
                    ret_addr <= 0;
                    ctxt_addr <= 0;
                    ret_addr_pc_rdy <= 0;
                    store_ctxt_finish <= 0;
                    matrix_select_reg <= matrix_select;
                    case (op_code_valid)
                        ADD: rst_InC <= 1;
                        SUB: rst_InC <= 1;
                        ABS: rst_InF <= 1;
                        TSC: rst_InF <= 1;
                        default:begin
                            rst_InF <= 0;
                            rst_InC <= 0;
                            input_C <= 0;
                            input_F <= 0;
                        end
                    endcase
                end
                LOAD_TMP: begin
                    case ({ctxt_rdy, tag_C_F})
                        2'b11: begin
                            bit_cnt <= tmp_bit_cnt_ret;
                            mask_reg <= tmp_mask_ret;
                            input_C <= tmp_C_F_ret;
                        end
                        2'b10: begin
                            bit_cnt <= tmp_bit_cnt_ret;
                            mask_reg <= tmp_mask_ret;
                            input_F <= tmp_C_F_ret;
                        end
                        default: ;
                    endcase
                end
                LOAD_CTXT: begin
                    if (ctrl_exp_7) begin
                        data_addr_tmp <= arith_4;
                    end
                    if (ctrl_exp_10) begin
                        ret_addr_pc <= ret_addr_ret;
                        ret_addr_pc_rdy <= 1;
                        rst_InC <= 0;
                        rst_InF <= 0;
                    end
                    if (ctrl_exp_11) begin
                        ret_addr_pc <= ret_addr_ret;
                        ret_addr_pc_rdy <= 1;
                        rst_InC <= 1;
                        rst_InF <= 1;
                    end
                end
                STORE_RBR: begin
                    data_addr_tmp <= data_addr;
                    case (matrix_select_reg)
                        M_A: data_out_rbr_tmp <= data_A_rbr;
                        M_B: data_out_rbr_tmp <= data_B_rbr;
                        M_R: data_out_rbr_tmp <= data_R_rbr;
                        default:;
                    endcase
                end
                STORE_CBC: begin
                    data_addr_tmp <= data_addr;
                    case (matrix_select_reg)
                        M_A: data_out_cbc_tmp <= data_A_cbc;
                        M_B: data_out_cbc_tmp <= data_B_cbc;
                        M_R: data_out_cbc_tmp <= data_R_cbc;
                        default:;
                    endcase
                end
                STORE_CTXT: begin
                    case (matrix_cnt)
                        1: data_out_cbc_tmp <= data_A_cbc;
                        2: data_out_cbc_tmp <= data_B_cbc;
                        3: begin
                            data_out_cbc_tmp <= data_R_cbc;
                            data_addr_tmp <= arith_4;
                        end
                        default:;
                    endcase
                end
                STORE_TMP: begin
                    tmp_bit_cnt <= bit_cnt;
                    tmp_pass <= pass;
                    tmp_mask <= mask_reg;
                    tmp_key_A <= key_A;
                    tmp_key_B <= key_B;
                    tmp_key_C <= key_C;
                    tmp_key_F <= key_F;
                    ret_addr <= addr_ins;
                    ctxt_addr <= addr_cur_ctxt;
                    case (opt_cur)
                        ADD: begin
                            tmp_C_F <= data_C;
                            tag_C_F <= 1;
                        end
                        SUB: begin
                            tmp_C_F <= data_C;
                            tag_C_F <= 1;
                        end
                        TSC: begin
                            tmp_C_F <= data_F;
                            tag_C_F <= 0;
                        end
                        ABS: begin
                            tmp_C_F <= data_F;
                            tag_C_F <= 0;
                        end
                        default: tmp_C_F <= 0;
                    endcase
                end
                STORE_CTXT_FC: begin
                    if (ctrl_exp_16) begin
                        store_ctxt_finish <= 1;
                    end
                end
                JMP_INS: begin
                    jmp_addr_pc <= jmp_addr;
                end
                PRINT_DATA: begin
                    data_addr_tmp <= addr_mem;
                end
                PASS_4_ADD: begin
                    if (ctrl_exp_3) begin
                        bit_cnt <= arith_6;
                    end
                end
                PASS_4_SUB: begin
                    if (ctrl_exp_3) begin
                        bit_cnt <= arith_6;
                    end
                end
                PASS_4_ABS: begin
                    if (ctrl_exp_3) begin
                        bit_cnt <= arith_6;
                    end
                end
                PASS_3_TSC: begin
                    if (ctrl_exp_3) begin
                        bit_cnt <= arith_6;
                    end
                end
                RET_STATE: begin
                    opt_cur <= op_code;
                end
                FINISH_CK: begin
                    if (ctrl_exp_9) begin
                        mask_reg <= arith_9;
                    end
                end 
                default:;
            endcase
        end
    end

    /* state transfer */
    always @ (*) begin
        case (st_cur)
            START: begin
                case (op_code_valid)
                    RESET: st_next = START;
                    RET: st_next = LOAD_TMP;
                    LOADRBR: st_next = LOAD_RBR;
                    LOADCBC: st_next = LOAD_CBC; 
                    COPY: st_next = COPY_MT; 
                    STORERBR: st_next = STORE_RBR; 
                    STORECBC: st_next = STORE_CBC; 
                    ADD: st_next = PASS_1_ADD; 
                    SUB: st_next = PASS_1_SUB; 
                    ABS: st_next = PASS_1_ABS; 
                    TSC: st_next = PASS_1_TSC; 
                    PRINT: st_next = PRINT_DATA; 
                    STOP: st_next = FINISH; 
                    default: st_next = START; 
                endcase
            end
            LOAD_RBR: begin
                case (data_cache_rdy)
                    1'b1: st_next = START; 
                    1'b0: st_next = LOAD_RBR; 
                    default: st_next = LOAD_RBR;
                endcase
            end
            LOAD_CBC:begin
                case (data_cache_rdy)
                    1'b1: st_next = START;
                    1'b0: st_next = LOAD_CBC;
                    default:st_next = LOAD_CBC;
                endcase
            end
            COPY_MT: begin
                case ({matrix_select_1, matrix_select_reg})
                    {M_A, M_B}: st_next = START;
                    {M_A, M_R}: st_next = START;
                    {M_B, M_A}: st_next = START;
                    {M_B, M_R}: st_next = START;
                    {M_R, M_A}: st_next = START;
                    {M_R, M_B}: st_next = START;
                    default: st_next = COPY_MT;
                endcase
            end
            STORE_RBR: begin
                case (ctrl_exp_3)
                    1'b1: st_next = STORE_END;
                    1'b0: st_next = STORE_RBR;
                    default: st_next = STORE_RBR;
                endcase
            end
            STORE_CBC: begin
                case (ctrl_exp_3)
                    1'b1: st_next = STORE_END;
                    1'b0: st_next = STORE_CBC;
                    default: st_next = STORE_CBC;
                endcase
            end
            STORE_END: begin
                st_next = STORE_END_2;
            end
            STORE_END_2: begin
                st_next = ctrl_exp_2? START: STORE_END;
            end
            STORE_TMP: begin
                st_next = STORE_CTXT;
            end
            STORE_CTXT: begin
                case ({ctrl_exp_4, ctrl_exp_5, ctrl_exp_6, 
                ctrl_exp_7, data_cache_rdy, ctrl_exp_3})
                    6'b100000: st_next = GET_JMP_ADDR;
                    6'b100001: st_next = GET_JMP_ADDR;
                    6'b100010: st_next = GET_JMP_ADDR;
                    6'b100011: st_next = GET_JMP_ADDR;
                    6'b010011: st_next = STORE_CTXT_FC;
                    6'b001011: st_next = STORE_CTXT_FC;
                    6'b000111: st_next = STORE_CTXT_FC;
                    default: st_next = STORE_CTXT;
                endcase
            end
            STORE_CTXT_FC: begin 
                st_next = ctrl_exp_14? STORE_CTXT : START;
            end
            GET_JMP_ADDR: begin
                st_next = jmp_addr_rdy? JMP_INS : GET_JMP_ADDR;
            end
            JMP_INS: begin
                st_next = START;
            end
            LOAD_TMP: begin
                st_next = data_cache_rdy? LOAD_CTXT : LOAD_TMP;
            end
            LOAD_CTXT: begin
                case ({ctrl_exp_4, ctrl_exp_5, ctrl_exp_6, 
                ctrl_exp_7, data_cache_rdy, ctrl_exp_8})
                    6'b100000: st_next = RET_STATE;
                    6'b100010: st_next = RET_STATE;
                    6'b010010: st_next = LOAD_CTXT_FC;
                    6'b010011: st_next = LOAD_CTXT_FC;
                    6'b001010: st_next = LOAD_CTXT_FC;
                    6'b001011: st_next = LOAD_CTXT_FC;
                    6'b000110: st_next = LOAD_CTXT_FC;
                    6'b000111: st_next = LOAD_CTXT_FC;
                    default: st_next = LOAD_CTXT;
                endcase
            end
            LOAD_CTXT_FC: begin
                st_next = ctrl_exp_14? LOAD_CTXT : START;
            end
            RET_STATE: begin
                case(op_code)
                    ADD: st_next = RSTTAG_ADD;
                    SUB: st_next = RSTTAG_SUB;
                    TSC: st_next = RSTTAG_TSC;
                    ABS: st_next = RSTTAG_ABS;
                    default: st_next = RET_STATE;
                endcase
            end
            PRINT_DATA: begin
                st_next = data_cache_rdy? START : PRINT_DATA;
            end
            /* pass of ADD */
            PASS_1_ADD: begin
                st_next = ctrl_exp_3? RSTTAG_ADD : PASS_1_ADD;
            end
                
            PASS_2_ADD: begin           
                st_next = ctrl_exp_3? RSTTAG_ADD : PASS_2_ADD;
            end
            PASS_3_ADD: begin
                st_next = ctrl_exp_3? RSTTAG_ADD : PASS_3_ADD;
            end
            PASS_4_ADD: begin
                st_next = ctrl_exp_3? RSTTAG_ADD : PASS_4_ADD;
            end
            RSTTAG_ADD: begin
                case(pass)
                    1: st_next = PASS_2_ADD;
                    2: st_next = PASS_3_ADD;
                    3: st_next = PASS_4_ADD;
                    4: st_next = FINISH_CK;
                    default: st_next = RSTTAG_ADD;
                endcase
            end       
            /* pass of SUB */
            PASS_1_SUB: begin
                st_next = ctrl_exp_3? RSTTAG_SUB : PASS_1_SUB;
            end
            PASS_2_SUB: begin
                st_next = ctrl_exp_3? RSTTAG_SUB : PASS_2_SUB;
            end
            PASS_3_SUB: begin
                st_next = ctrl_exp_3? RSTTAG_SUB : PASS_3_SUB;
            end
            PASS_4_SUB: begin
                st_next = ctrl_exp_3? RSTTAG_SUB : PASS_4_SUB;
            end
            RSTTAG_SUB: begin
                case(pass)
                    1: st_next = PASS_2_SUB;
                    2: st_next = PASS_3_SUB;
                    3: st_next = PASS_4_SUB;
                    4: st_next = FINISH_CK;
                    default: st_next = RSTTAG_SUB;
                endcase
            end
            /* pass of ABS */
            PASS_1_ABS: begin
                st_next = ctrl_exp_3? RSTTAG_ABS : PASS_1_ABS;
            end
            PASS_2_ABS: begin
                st_next = ctrl_exp_3? RSTTAG_ABS : PASS_2_ABS;
            end
            PASS_3_ABS: begin
                st_next = ctrl_exp_3? RSTTAG_ABS : PASS_3_ABS;
            end
            PASS_4_ABS: begin
                st_next = ctrl_exp_3? RSTTAG_ABS : PASS_4_ABS;
            end
            RSTTAG_ABS: begin
                case(pass)
                    1: st_next = PASS_2_ABS;
                    2: st_next = PASS_3_ABS;
                    3: st_next = PASS_4_ABS;
                    4: st_next = FINISH_CK;
                    default: st_next = RSTTAG_ABS;
                endcase
            end
            /* pass of TSC */        
            PASS_1_TSC: begin
                st_next = ctrl_exp_3? RSTTAG_TSC : PASS_1_TSC;
            end
            PASS_2_TSC: begin
                st_next = ctrl_exp_3? RSTTAG_TSC : PASS_2_TSC;
            end
            PASS_3_TSC: begin
                st_next = ctrl_exp_3? RSTTAG_TSC : PASS_3_TSC;
            end
            RSTTAG_TSC: begin
                case(pass)
                    1: st_next = PASS_2_TSC;
                    2: st_next = PASS_3_TSC;
                    3: st_next = FINISH_CK;
                    default: st_next = RSTTAG_TSC;
                endcase
            end
            FINISH_CK: begin
                if(ctrl_exp_9) begin
                    case(opt_cur)
                        ADD: st_next = PASS_1_ADD;
                        SUB: st_next = PASS_1_SUB;
                        TSC: st_next = PASS_1_TSC;
                        ABS: st_next = PASS_1_ABS;
                        default: st_next = FINISH_CK;
                    endcase
                end
                else begin
                    st_next = START;
                end
            end
            FINISH: begin
                st_next = FINISH;
            end
            default: begin
                st_next = START;
            end
        endcase
    end

    always @ (*) begin
        case (st_cur)
            START: begin
                pass = 0;
                key_A = 0;
                key_B = 0;
                key_C = 0;
                key_F = 0;
                rst_tag = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_mem_col = 0;
                data_cmd = 0;
                ret_valid = 0;
                int_set = 0;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = DATA_DEPTH_P_3;
                addr_output_rbr_B = DATA_DEPTH_P_3;
                addr_output_rbr_R = DATA_DEPTH_P_3;
                addr_output_cbc_A = DATA_WIDTH_P_3;
                addr_output_cbc_B = DATA_WIDTH_P_3;
                addr_output_cbc_R = DATA_WIDTH_P_3;
                data_out_cbc = data_out_cbc_tmp;
                data_out_rbr = data_out_rbr_tmp;
                inout_mode = 0;
                data_addr = data_addr_tmp;
                addr_cam_col = addr_cam_tmp;
                data_print_rdy = 0;
                data_print = 0;
                case (op_code_valid)
                    RESET: ins_inp_valid = 1;
                    RET: ins_inp_valid = 1;
                    LOADRBR: ins_inp_valid = 0;
                    LOADCBC: ins_inp_valid = 0;
                    COPY: ins_inp_valid = 1;
                    STORERBR: ins_inp_valid = 0;
                    STORECBC: ins_inp_valid = 0;
                    ADD: ins_inp_valid = 0;
                    SUB: ins_inp_valid = 0;
                    ABS: begin
                        ins_inp_valid = 0;
                        inout_mode = RST0;
                    end
                    TSC: begin
                        ins_inp_valid = 0;
                        inout_mode = RST0;
                    end
                    PRINT: ins_inp_valid = 0;
                    STOP: ins_inp_valid = 0;
                    default: ins_inp_valid = 1;
                endcase
            end
            LOAD_RBR: begin
                inout_mode = RowxRow;
                data_addr = addr_mem;
                data_cmd = RowxRow_load;
                pass = 0;
                key_A = 0;
                key_B = 0;
                key_C = 0;
                key_F = 0;
                rst_tag = 0;
                abs_opt = 0;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
                case ({matrix_select_reg, data_cache_rdy})
                    {M_A, 1'b1}: begin
                        rst_InA = 0;
                        rst_InB = 1;
                        rst_InR = 1;
                        addr_input_rbr_A = addr_cam;
                        addr_input_rbr_B = 0;
                        addr_input_rbr_R = 0;
                        ins_inp_valid = 1;
                        input_A_rbr = data_in_rbr;
                        input_B_rbr = 0;
                        input_R_rbr = 0;
                    end
                    {M_A, 1'b0}: begin
                        rst_InA = 0;
                        rst_InB = 1;
                        rst_InR = 1;
                        addr_input_rbr_A = addr_cam;
                        addr_input_rbr_B = 0;
                        addr_input_rbr_R = 0;
                        ins_inp_valid = 0;
                        input_A_rbr = 0;
                        input_B_rbr = 0;
                        input_R_rbr = 0;
                    end
                    {M_B, 1'b1}: begin
                        rst_InA = 1;
                        rst_InB = 0;
                        rst_InR = 1;
                        addr_input_rbr_B = addr_cam;
                        addr_input_rbr_A = 0;
                        addr_input_rbr_R = 0;
                        ins_inp_valid = 1;
                        input_B_rbr = data_in_rbr;
                        input_A_rbr = 0;
                        input_R_rbr = 0;
                    end
                    {M_B, 1'b0}: begin
                        rst_InA = 1;
                        rst_InB = 0;
                        rst_InR = 1;
                        addr_input_rbr_B = addr_cam;
                        addr_input_rbr_A = 0;
                        addr_input_rbr_R = 0;
                        ins_inp_valid = 0;
                        input_A_rbr = 0;
                        input_B_rbr = 0;
                        input_R_rbr = 0;
                    end
                    {M_R, 1'b1}: begin
                        rst_InA = 1;
                        rst_InB = 1;
                        rst_InR = 0;
                        addr_input_rbr_R = addr_cam;
                        addr_input_rbr_A = 0;
                        addr_input_rbr_B = 0;
                        ins_inp_valid = 1;
                        input_R_rbr = data_in_rbr;
                        input_A_rbr = 0;
                        input_B_rbr = 0;
                    end
                    {M_R, 1'b0}: begin
                        rst_InA = 1;
                        rst_InB = 1;
                        rst_InR = 0;
                        addr_input_rbr_R = addr_cam;
                        addr_input_rbr_A = 0;
                        addr_input_rbr_B = 0;
                        ins_inp_valid = 0;
                        input_A_rbr = 0;
                        input_B_rbr = 0;
                        input_R_rbr = 0;
                    end
                    default: begin
                        ins_inp_valid = 0;
                        rst_InA = 1;
                        rst_InB = 1;
                        rst_InR = 1;
                        addr_input_rbr_A = 0;
                        addr_input_rbr_B = 0;
                        addr_input_rbr_R = 0;
                        input_A_rbr = 0;
                        input_B_rbr = 0;
                        input_R_rbr = 0;
                    end
                endcase
            end
            LOAD_CBC: begin
                inout_mode = ColxCol;
                data_addr = addr_mem;
                data_cmd = ColxCol_load;
                addr_cam_col = addr_cam;
                pass = 0;
                key_A = 0;
                key_B = 0;
                key_C = 0;
                key_F = 0;
                rst_tag = 0;
                abs_opt = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                data_print_rdy = 0;
                data_print = 0;
                case ({matrix_select_reg, data_cache_rdy})
                    {M_A, 1'b1}: begin
                        rst_InA = 0;
                        rst_InB = 1;
                        rst_InR = 1;
                        addr_input_cbc_A = addr_cam;
                        addr_input_cbc_B = 0;
                        addr_input_cbc_R = 0;
                        input_A_cbc = data_in_cbc;
                        input_B_cbc = 0;
                        input_R_cbc = 0;
                        ins_inp_valid = 1;
                    end
                    {M_A, 1'b0}: begin
                        rst_InA = 0;
                        rst_InB = 1;
                        rst_InR = 1;
                        addr_input_cbc_A = addr_cam;
                        addr_input_cbc_B = 0;
                        addr_input_cbc_R = 0;
                        input_A_cbc = 0;
                        input_B_cbc = 0;
                        input_R_cbc = 0;
                        ins_inp_valid = 0;
                    end
                    {M_B, 1'b1}: begin
                        rst_InA = 1;
                        rst_InB = 0;
                        rst_InR = 1;
                        addr_input_cbc_B = addr_cam;
                        addr_input_cbc_A = 0;
                        addr_input_cbc_R = 0;
                        input_B_cbc = data_in_cbc;
                        input_A_cbc = 0;
                        input_R_cbc = 0;
                        ins_inp_valid = 1;
                    end
                    {M_B, 1'b0}: begin
                        rst_InA = 1;
                        rst_InB = 0;
                        rst_InR = 1;
                        addr_input_cbc_B = addr_cam;
                        addr_input_cbc_A = 0;
                        addr_input_cbc_R = 0;
                        ins_inp_valid = 0;
                        input_A_cbc = 0;
                        input_B_cbc = 0;
                        input_R_cbc = 0;
                    end
                    {M_R, 1'b1}:begin
                        rst_InA = 1;
                        rst_InB = 1;
                        rst_InR = 0;
                        addr_input_cbc_R = addr_cam;
                        addr_input_cbc_A = 0;
                        addr_input_cbc_B = 0;
                        input_R_cbc = data_in_cbc;
                        input_A_cbc = 0;
                        input_B_cbc = 0;
                        ins_inp_valid = 1;
                    end
                    {M_R, 1'b0}:begin
                        rst_InA = 1;
                        rst_InB = 1;
                        rst_InR = 0;
                        addr_input_cbc_R = addr_cam;
                        addr_input_cbc_A = 0;
                        addr_input_cbc_B = 0;
                        ins_inp_valid = 0;
                        input_A_cbc = 0;
                        input_B_cbc = 0;
                        input_R_cbc = 0;
                    end
                    default:begin    
                        ins_inp_valid = 0;
                        rst_InA = 1;
                        rst_InB = 1;
                        rst_InR = 1;
                        addr_input_cbc_A = 0;
                        addr_input_cbc_B = 0;
                        addr_input_cbc_R = 0;
                        input_A_cbc = 0;
                        input_B_cbc = 0;
                        input_R_cbc = 0;
                    end
                endcase
            end
            COPY_MT: begin
                ins_inp_valid = 1;
                pass = 0;
                key_A = 0;
                key_B = 0;
                key_C = 0;
                key_F = 0;
                rst_tag = 0;
                abs_opt = 0;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
                case ({matrix_select_1, matrix_select_reg})
                    {M_A, M_B}: begin
                        rst_InA = 0;
                        rst_InB = 1;
                        rst_InR = 1;
                        inout_mode = COPY_B;
                    end
                    {M_A, M_R}: begin
                        rst_InA = 0;
                        rst_InB = 1;
                        rst_InR = 1;
                        inout_mode = COPY_R;
                    end

                    {M_B, M_A}: begin
                        rst_InA = 1;
                        rst_InB = 0;
                        rst_InR = 1;
                        inout_mode = COPY_A;
                    end
                    {M_B, M_R}: begin
                        rst_InA = 1;
                        rst_InB = 0;
                        rst_InR = 1;
                        inout_mode = COPY_R;
                    end

                    {M_R, M_A}: begin
                        rst_InA = 1;
                        rst_InB = 1;
                        rst_InR = 0;
                        inout_mode = COPY_A;
                    end
                    {M_R, M_B}: begin
                        rst_InA = 1;
                        rst_InB = 1;
                        rst_InR = 0;
                        inout_mode = COPY_B;
                    end
                    default: begin
                        rst_InA = 1;
                        rst_InB = 1;
                        rst_InR = 1;
                        inout_mode = 0;
                    end
                endcase
            end
            STORE_RBR: begin
                inout_mode = RowxRow;
                data_addr = addr_mem;
                data_cmd = RowxRow_store;
                ins_inp_valid = 0;
                pass = 0;
                key_A = 0;
                key_B = 0;
                key_C = 0;
                key_F = 0;
                rst_tag = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
                case (matrix_select_reg)
                    M_B: begin
                        addr_output_rbr_B = addr_cam;
                        addr_output_rbr_A = 0;
                        addr_output_rbr_R = 0;
                        data_out_rbr = data_B_rbr;
                    end
                    M_R: begin
                        addr_output_rbr_R = addr_cam;
                        addr_output_rbr_A = 0;
                        addr_output_rbr_B = 0;
                        data_out_rbr = data_R_rbr;
                    end
                    M_A: begin
                        addr_output_rbr_A = addr_cam;
                        addr_output_rbr_B = 0;
                        addr_output_rbr_R = 0;
                        data_out_rbr = data_A_rbr;
                    end
                    default: begin
                        addr_output_rbr_A = DATA_DEPTH_P_3;
                        addr_output_rbr_B = DATA_DEPTH_P_3;
                        addr_output_rbr_R = DATA_DEPTH_P_3;
                        data_out_rbr = 0;
                    end
                endcase
            end
            STORE_CBC: begin
                inout_mode = ColxCol;
                data_addr = addr_mem;
                data_cmd = ColxCol_store;
                ins_inp_valid = 0;
                addr_cam_col = addr_cam;
                pass = 0;
                key_A = 0;
                key_B = 0;
                key_C = 0;
                key_F = 0;
                rst_tag = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                data_out_rbr = 0;
                ret_valid = 0;
                int_set = 0;
                data_print_rdy = 0;
                data_print = 0;
                case (matrix_select_reg)
                    M_A: begin
                        addr_output_cbc_A = addr_cam_col;
                        addr_output_cbc_B = 0;
                        addr_output_cbc_R = 0;
                        data_out_cbc = data_A_cbc;
                    end
                    M_B: begin
                        addr_output_cbc_B = addr_cam_col;
                        addr_output_cbc_A = 0;
                        addr_output_cbc_R = 0;
                        data_out_cbc = data_B_cbc;
                    end
                    M_R: begin
                        addr_output_cbc_R = addr_cam_col;
                        addr_output_cbc_A = 0;
                        addr_output_cbc_B = 0;
                        data_out_cbc = data_R_cbc;
                    end
                    default: begin
                        addr_output_cbc_A = DATA_WIDTH_P_3;
                        addr_output_cbc_B = DATA_WIDTH_P_3;
                        addr_output_cbc_R = DATA_WIDTH_P_3;
                        data_out_cbc = 0;
                    end
                endcase
            end
            STORE_END: begin
                pass = pass_tmp;
                key_A = key_A_tmp;
                key_B = key_B_tmp;
                key_C = key_C_tmp;
                key_F = key_F_tmp;
                rst_tag = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
                ins_inp_valid = 0;//ctrl_exp_2;
            end
            STORE_END_2: begin
                pass = pass_tmp;
                key_A = key_A_tmp;
                key_B = key_B_tmp;
                key_C = key_C_tmp;
                key_F = key_F_tmp;
                rst_tag = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
                ins_inp_valid = ctrl_exp_2;
            end
            STORE_TMP: begin
                pass = pass_tmp;
                key_A = key_A_tmp;
                key_B = key_B_tmp;
                key_C = key_C_tmp;
                key_F = key_F_tmp;
                ins_inp_valid = 0;
                rst_tag = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                int_set = 1;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
            end
            STORE_CTXT: begin
                inout_mode = ColxCol;
                data_cmd = ColxCol_store;
                ins_inp_valid = 0;
                addr_cam_col = addr_cam_auto;
                pass = 0;
                key_A = 0;
                key_B = 0;
                key_C = 0;
                key_F = 0;
                rst_tag = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                data_out_rbr = 0;
                ret_valid = 0;
                int_set = 1;
                data_print_rdy = 0;
                data_print = 0;
                case ({ctrl_exp_4, ctrl_exp_5,
                       ctrl_exp_6, ctrl_exp_7})
                    4'b1000: begin
                        addr_output_cbc_A = 0;
                        addr_output_cbc_B = 0;
                        addr_output_cbc_R = 0;
                        data_out_cbc = 0;
                        data_addr = data_addr_tmp;
                    end
                    4'b0100: begin
                        addr_output_cbc_A = addr_cam_col;
                        addr_output_cbc_B = 0;
                        addr_output_cbc_R = 0;
                        data_out_cbc = data_A_cbc;
                        data_addr = addr_cur_ctxt;
                    end
                    4'b0010: begin
                        addr_output_cbc_B = addr_cam_col;
                        addr_output_cbc_A = 0;
                        addr_output_cbc_R = 0;
                        data_out_cbc = data_B_cbc;
                        data_addr = arith_7;
                    end
                    4'b0001: begin
                        addr_output_cbc_R = addr_cam_col;
                        addr_output_cbc_A = 0;
                        addr_output_cbc_B = 0;
                        data_out_cbc = data_R_cbc;
                        data_addr = arith_8;
                    end
                    default: begin
                        addr_output_cbc_A = 0;
                        addr_output_cbc_B = 0;
                        addr_output_cbc_R = 0;
                        data_out_cbc = 0;
                        data_addr = data_addr_tmp;
                    end
                endcase
            end
            STORE_CTXT_FC: begin
                ins_inp_valid = 0;
                pass = 0;
                key_A = 0;
                key_B = 0;
                key_C = 0;
                key_F = 0;
                rst_tag = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = data_out_cbc_tmp;
                ret_valid = 0;
                int_set = 1;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = data_addr_tmp;
                addr_cam_col = addr_cam_auto;
                data_print_rdy = 0;
                data_print = 0;
            end
            GET_JMP_ADDR: begin
                ins_inp_valid = 0;
                inout_mode = RowxRow;
                data_addr = 16'he001;
                data_cmd = Addr_load;
                pass = 0;
                key_A = 0;
                key_B = 0;
                key_C = 0;
                key_F = 0;
                rst_tag = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 1;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
            end
            JMP_INS: begin
                ins_inp_valid = 0;
                data_cmd = 0;
                pass = 0;
                key_A = 0;
                key_B = 0;
                key_C = 0;
                key_F = 0;
                rst_tag = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 1;
                inout_mode = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
            end
            LOAD_TMP: begin
                ins_inp_valid = 0;
                ret_valid = 1;
                rst_tag = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
                if (ctxt_rdy == 1) begin
                    pass = tmp_pass_ret;
                    key_A = tmp_key_A_ret;
                    key_B = tmp_key_B_ret;
                    key_C = tmp_key_C_ret;
                    key_F = tmp_key_F_ret;
                end
                else begin
                    pass = pass_tmp;
                    key_A = key_A_tmp;
                    key_B = key_B_tmp;
                    key_C = key_C_tmp;
                    key_F = key_F_tmp;
                end
            end
            LOAD_CTXT:
                begin
                inout_mode = ColxCol;
                ins_inp_valid = 0;
                addr_cam_col = addr_cam_auto;
                pass = pass_tmp;
                key_A = key_A_tmp;
                key_B = key_B_tmp;
                key_C = key_C_tmp;
                key_F = key_F_tmp;
                rst_tag = 0;
                abs_opt = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 1;
                int_set = 0;
                data_print_rdy = 0;
                data_print = 0;
                case ({ctrl_exp_4, ctrl_exp_5,
                       ctrl_exp_6, ctrl_exp_7,
                       data_cache_rdy})
                    5'b10000: begin
                        data_cmd = 0;
                        rst_InA = 1;
                        rst_InB = 1;
                        rst_InR = 1;
                        addr_input_cbc_A = 0;
                        addr_input_cbc_B = 0;
                        addr_input_cbc_R = 0;
                        input_A_cbc = 0;
                        input_B_cbc = 0;
                        input_R_cbc = 0;
                        data_addr = data_addr_tmp;
                    end
                    5'b10001: begin
                        data_cmd = 0;
                        rst_InA = 1;
                        rst_InB = 1;
                        rst_InR = 1;
                        addr_input_cbc_A = 0;
                        addr_input_cbc_B = 0;
                        addr_input_cbc_R = 0;
                        input_A_cbc = 0;
                        input_B_cbc = 0;
                        input_R_cbc = 0;
                        data_addr = data_addr_tmp;
                    end
                    5'b01000: begin
                        rst_InA = 0;
                        rst_InB = 1;
                        rst_InR = 1;
                        addr_input_cbc_A = addr_cam_col;
                        addr_input_cbc_B = 0;
                        addr_input_cbc_R = 0;
                        data_addr = ctxt_addr_ret;
                        data_cmd = ColxCol_load;
                        input_A_cbc = 0;
                        input_B_cbc = 0;
                        input_R_cbc = 0;
                    end
                    5'b01001: begin
                        rst_InA = 0;
                        rst_InB = 1;
                        rst_InR = 1;
                        addr_input_cbc_A = addr_cam_col;
                        addr_input_cbc_B = 0;
                        addr_input_cbc_R = 0;
                        data_addr = ctxt_addr_ret;
                        data_cmd = ColxCol_load;
                        input_A_cbc = data_in_cbc;
                        input_B_cbc = 0;
                        input_R_cbc = 0;
                    end
                    5'b00100: begin
                        rst_InA = 1;
                        rst_InB = 0;
                        rst_InR = 1;
                        addr_input_cbc_B = addr_cam_col;
                        addr_input_cbc_A = 0;
                        addr_input_cbc_R = 0;
                        data_addr = arith_5;
                        data_cmd = ColxCol_load;
                        input_A_cbc = 0;
                        input_B_cbc = 0;
                        input_R_cbc = 0;
                    end
                    5'b00101: begin
                        rst_InA = 1;
                        rst_InB = 0;
                        rst_InR = 1;
                        addr_input_cbc_B = addr_cam_col;
                        addr_input_cbc_A = 0;
                        addr_input_cbc_R = 0;
                        data_addr = arith_5;
                        data_cmd = ColxCol_load;
                        input_B_cbc = data_in_cbc;
                        input_A_cbc = 0;
                        input_R_cbc = 0;
                    end
                    5'b00010: begin
                        rst_InA = 1;
                        rst_InB = 1;
                        rst_InR = 0;
                        addr_input_cbc_R = addr_cam_col;
                        addr_input_cbc_A = 0;
                        addr_input_cbc_B = 0;
                        data_addr = arith_4;
                        data_cmd = ColxCol_load;
                        input_A_cbc = 0;
                        input_B_cbc = 0;
                        input_R_cbc = 0;
                    end
                    5'b00011: begin
                        rst_InA = 1;
                        rst_InB = 1;
                        rst_InR = 0;
                        addr_input_cbc_R = addr_cam_col;
                        addr_input_cbc_A = 0;
                        addr_input_cbc_B = 0;
                        data_addr = arith_4;
                        data_cmd = ColxCol_load;
                        input_R_cbc = data_in_cbc;
                        input_A_cbc = 0;
                        input_B_cbc = 0;
                    end
                    default: begin
                        rst_InA = 1;
                        rst_InB = 1;
                        rst_InR = 1;
                        addr_input_cbc_A = 0;
                        addr_input_cbc_B = 0;
                        addr_input_cbc_R = 0;
                        input_A_cbc = 0;
                        input_B_cbc = 0;
                        input_R_cbc = 0;
                        data_cmd = ColxCol_load;
                        data_addr = data_addr_tmp;
                    end
                endcase
            end
            LOAD_CTXT_FC: begin
                ins_inp_valid = 0;
                pass = pass_tmp;
                key_A = key_A_tmp;
                key_B = key_B_tmp;
                key_C = key_C_tmp;
                key_F = key_F_tmp;
                rst_tag = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 1;
                int_set = 0;
                inout_mode = ColxCol;
                data_cmd = ColxCol_load;
                data_addr = data_addr_tmp;
                addr_cam_col = addr_cam_auto;
                data_print_rdy = 0;
                data_print = 0;
            end
            RET_STATE: begin
                ins_inp_valid = 0;
                pass = pass_tmp;
                key_A = key_A_tmp;
                key_B = key_B_tmp;
                key_C = key_C_tmp;
                key_F = key_F_tmp;
                rst_tag = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = data_addr_tmp;
                addr_cam_col = addr_cam_auto;
                data_print_rdy = 0;
                data_print = 0;
            end
            PRINT_DATA: begin
                inout_mode = RowxRow;
                data_addr = addr_mem;
                data_cmd = RowxRow_load;
                pass = 0;
                key_A = 0;
                key_B = 0;
                key_C = 0;
                key_F = 0;
                rst_tag = 0;
                abs_opt = 0;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                addr_cam_col = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                if (data_cache_rdy == 1) begin
                    ins_inp_valid = 1;
                    data_print_rdy = 1;
                    data_print = data_in_rbr;
                end
                else begin
                    ins_inp_valid = 0;
                    data_print_rdy = 0;
                    data_print = 0;
                end
            end
            /* pass of ADD */
            PASS_1_ADD: begin
                ins_inp_valid = 0;
                rst_tag = 1;
                pass = 1;
                key_A = 1;
                key_B = 1;
                key_C = 0;
                key_F = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
            end    
            PASS_2_ADD: begin
                ins_inp_valid = 0;
                rst_tag = 1;
                pass = 2;
                key_A = 1;
                key_B = 0;
                key_C = 0;
                key_F = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_addr = 0;
                data_cmd = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
            end
            PASS_3_ADD: begin
                ins_inp_valid = 0;
                rst_tag = 1;
                pass = 3;
                key_A = 0;
                key_B = 0;
                key_C = 1;
                key_F = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
            end
            PASS_4_ADD: begin
                ins_inp_valid = 0;
                rst_tag = 1;
                pass = 4;
                key_A = 0;
                key_B = 1;
                key_C = 1;
                key_F = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
            end
            RSTTAG_ADD: begin
                ins_inp_valid = 0;
                rst_tag = 0;
                pass = pass_tmp;
                key_A = 0;
                key_B = 0;
                key_C = 0;
                key_F = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
            end       
            /* pass of SUB */
            PASS_1_SUB: begin
                ins_inp_valid = 0;
                rst_tag = 1;
                pass = 1;
                key_A = 1;
                key_B = 0;
                key_C = 0;
                key_F = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
            end
            PASS_2_SUB: begin
                ins_inp_valid = 0;
                rst_tag = 1;
                pass = 2;
                key_A = 1;
                key_B = 1;
                key_C = 0;
                key_F = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
            end
            PASS_3_SUB: begin
                ins_inp_valid = 0;
                rst_tag = 1;
                pass = 3;
                key_A = 0;
                key_B = 1;
                key_C = 1;
                key_F = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
            end
            PASS_4_SUB: begin
                ins_inp_valid = 0;
                rst_tag = 1;
                pass = 4;
                key_A = 0;
                key_B = 0;
                key_C = 1;
                key_F = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
            end
            RSTTAG_SUB: begin
                rst_tag = 0;
                ins_inp_valid = 0;
                pass = pass_tmp;
                key_A = 0;
                key_B = 0;
                key_C = 0;
                key_F = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
            end
            /* pass of ABS */
            PASS_1_ABS: begin
                ins_inp_valid = 0;
                rst_tag = 1;
                pass = 1;
                key_A = 1;
                key_B = 0;
                key_C = 0;
                key_F = 0;
                abs_opt = 1;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
            end
            PASS_2_ABS: begin
                ins_inp_valid = 0;
                rst_tag = 1;
                pass = 2;
                key_A = 0;
                key_B = 0;
                key_C = 0;
                key_F = 1;
                abs_opt = 1;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
            end
            PASS_3_ABS: begin
                ins_inp_valid = 0;
                rst_tag = 1;
                pass = 3;
                key_A = 1;
                key_B = 0;
                key_C = 0;
                key_F = 1;
                abs_opt = 1;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
            end
            PASS_4_ABS: begin
                ins_inp_valid = 0;
                rst_tag = 1;
                pass = 4;
                key_A = 1;
                key_B = 0;
                key_C = 0;
                key_F = 0;
                abs_opt = 1;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
            end
            RSTTAG_ABS: begin
                rst_tag = 0; 
                ins_inp_valid = 0;
                pass = pass_tmp;
                key_A = 0;
                key_B = 0;
                key_C = 0;
                key_F = 0;
                abs_opt = 1;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
            end
            /* pass of TSC */        
            PASS_1_TSC: begin
                ins_inp_valid = 0;
                rst_tag = 1;
                pass = 1;
                key_A = 0;
                key_B = 0;
                key_C = 0;
                key_F = 1;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
            end
            PASS_2_TSC: begin
                ins_inp_valid = 0;
                rst_tag = 1;
                pass = 2;
                key_A = 1;
                key_B = 0;
                key_C = 0;
                key_F = 1;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
            end
            PASS_3_TSC: begin
                ins_inp_valid = 0;
                rst_tag = 1;
                pass = 3;
                key_A = 1;
                key_B = 0;
                key_C = 0;
                key_F = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
            end
            RSTTAG_TSC: begin
                rst_tag = 0; 
                ins_inp_valid = 0; 
                pass = pass_tmp;
                key_A = 0;
                key_B = 0;
                key_C = 0;
                key_F = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
            end
            FINISH_CK: begin
                pass = pass_tmp;
                key_A = key_A_tmp;
                key_B = key_B_tmp;
                key_C = key_C_tmp;
                key_F = key_F_tmp;
                rst_tag = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
                if(ctrl_exp_9) begin
                    ins_inp_valid = 0;
                end
                else begin
                    ins_inp_valid = 1;
                end
            end
            FINISH: begin
                ins_inp_valid = 0;
                pass = 0;
                key_A = 0;
                key_B = 0;
                key_C = 0;
                key_F = 0;
                rst_tag = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
            end
            default: begin
                ins_inp_valid = 0;
                pass = pass_tmp;
                key_A = key_A_tmp;
                key_B = key_B_tmp;
                key_C = key_C_tmp;
                key_F = key_F_tmp;
                rst_tag = 0;
                abs_opt = 0;
                rst_InA = 1;
                rst_InB = 1;
                rst_InR = 1;
                addr_input_cbc_A = 0;
                addr_input_cbc_B = 0;
                addr_input_cbc_R = 0;
                addr_input_rbr_A = 0;
                addr_input_rbr_B = 0;
                addr_input_rbr_R = 0;
                input_A_rbr = 0;
                input_B_rbr = 0;
                input_R_rbr = 0;
                input_A_cbc = 0;
                input_B_cbc = 0;
                input_R_cbc = 0;
                addr_output_rbr_A = 0;
                addr_output_rbr_B = 0;
                addr_output_rbr_R = 0;
                addr_output_cbc_A = 0;
                addr_output_cbc_B = 0;
                addr_output_cbc_R = 0;
                data_out_rbr = 0;
                data_out_cbc = 0;
                ret_valid = 0;
                int_set = 0;
                inout_mode = 0;
                data_cmd = 0;
                data_addr = 0;
                addr_cam_col = 0;
                data_print_rdy = 0;
                data_print = 0;
            end
        endcase
    end

endmodule