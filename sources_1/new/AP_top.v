module AP_top
#(
    parameter DATA_WIDTH        = 8,
    parameter DATA_DEPTH        = 16,
    parameter ISA_DEPTH         = 72,
    parameter INT_INS_DEPTH     = 27,
    parameter TOTAL_ISA_DEPTH   = 128,
    parameter TOTAL_DATA_DEPTH  = 128,
    parameter DATA_CACHE_DEPTH  = 16,
    parameter DDR_ADDR_WIDTH    = 28,
    parameter DDR_DATA_WIDTH    = 128,
    parameter OPCODE_WIDTH      = 4,
    parameter ADDR_WIDTH_CAM    = 8,
    parameter OPRAND_2_WIDTH    = 2,
    parameter ADDR_WIDTH_MEM    = 16,
    parameter ISA_WIDTH         = OPCODE_WIDTH 
                                + ADDR_WIDTH_CAM
                                + OPRAND_2_WIDTH 
                                + ADDR_WIDTH_MEM
   )
    (
    /* interface of system */
    input                               sys_clk_i_1,
    input                               sys_clk_i_2,
    input                               int,
    output                              init_calib_complete,
    input [ISA_WIDTH - 1 : 0]           ins_input,
    input [DATA_WIDTH - 1 : 0]          data_input,
    input			                    sys_rst,
    output                              wr_burst_data_req, 
    output                              load_ins_ddr,
	output                              load_data_ddr,
	output                              load_int_ins_ddr,
    output [DATA_WIDTH - 1 : 0]         data_print,
    output                              data_print_rdy,
    output                              finish_flag,
    
    /* interface of DDR3 */
    inout [15:0]                        ddr3_dq,
    inout [1:0]                         ddr3_dqs_n,
    inout [1:0]                         ddr3_dqs_p,
    output [13:0]                       ddr3_addr,
    output [2:0]                        ddr3_ba,
    output                              ddr3_ras_n,
    output                              ddr3_cas_n,
    output                              ddr3_we_n,
    output                              ddr3_reset_n,
    output [0:0]                        ddr3_ck_p,
    output [0:0]                        ddr3_ck_n,
    output [0:0]                        ddr3_cke,
    output [0:0]                        ddr3_cs_n,
    output [1:0]                        ddr3_dm,
    output [0:0]                        ddr3_odt
    );

    /*localparam                          ADDR_WIDTH_R = ADDR_WIDTH_CAM;
    localparam                          ADDR_WIDTH_C = ADDR_WIDTH_CAM;*/ 
    
    wire                                ui_clk;
    wire [ADDR_WIDTH_CAM - 1 : 0]       addr_cam_col;
    wire [DATA_WIDTH - 1 : 0]           data_out_rbr;
    wire [DATA_DEPTH - 1 : 0]           data_out_cbc;
    wire [ADDR_WIDTH_MEM - 1 : 0]       data_addr;
    wire [2 : 0]                        data_cmd;
    wire                                store_ddr_en;
    wire                                store_ctxt_finish;
    wire                                load_ctxt_finish;
    wire                                ins_inp_valid;
    wire [ADDR_WIDTH_MEM - 1 : 0]       ret_addr_pc;
    wire                                ret_addr_pc_rdy;
    wire                                int_set;
    wire                                ret_valid;
    wire [ADDR_WIDTH_MEM - 1 : 0]       ret_addr;
    wire [ADDR_WIDTH_MEM - 1 : 0]       ctxt_addr;
    wire [DATA_WIDTH - 1 : 0]           tmp_bit_cnt;
    wire [2 : 0]                        tmp_pass;
    wire [DATA_WIDTH - 1 : 0]           tmp_mask;
    wire [DATA_DEPTH - 1 : 0]           tmp_C_F;
    wire                                tmp_key_A;
    wire                                tmp_key_B;
    wire                                tmp_key_C;
    wire                                tmp_key_F;
    wire                                tmp_key_A_ret;
    wire                                tmp_key_B_ret;
    wire                                tmp_key_C_ret;
    wire                                tmp_key_F_ret;
    wire [DATA_WIDTH - 1 : 0]           input_A_rbr;
    wire [DATA_DEPTH - 1 : 0]           input_A_cbc;
    wire [ADDR_WIDTH_CAM - 1 : 0]       addr_input_rbr_A;
    wire [ADDR_WIDTH_CAM - 1 : 0]       addr_input_cbc_A;
    wire [ADDR_WIDTH_CAM - 1 : 0]       addr_output_rbr_A;
    wire [ADDR_WIDTH_CAM - 1 : 0]       addr_output_cbc_A;
    wire [DATA_WIDTH - 1 : 0]           input_B_rbr;
    wire [DATA_DEPTH - 1 : 0]           input_B_cbc;
    wire [ADDR_WIDTH_CAM - 1 : 0]       addr_input_rbr_B;
    wire [ADDR_WIDTH_CAM - 1 : 0]       addr_input_cbc_B;
    wire [ADDR_WIDTH_CAM - 1 : 0]       addr_output_rbr_B;
    wire [ADDR_WIDTH_CAM - 1 : 0]       addr_output_cbc_B;
    wire [DATA_WIDTH - 1 : 0]           input_R_rbr;
    wire [DATA_DEPTH - 1 : 0]           input_R_cbc;
    wire [ADDR_WIDTH_CAM - 1 : 0]       addr_input_rbr_R;
    wire [ADDR_WIDTH_CAM - 1 : 0]       addr_input_cbc_R;
    wire [ADDR_WIDTH_CAM - 1 : 0]       addr_output_rbr_R;
    wire [ADDR_WIDTH_CAM - 1 : 0]       addr_output_cbc_R;
    wire [DATA_DEPTH - 1 : 0]           input_C;
    wire [DATA_DEPTH - 1 : 0]           input_F;
    wire                                abs_opt;
    wire                                rst_InA;
    wire                                rst_InB;
    wire                                rst_InC;
    wire                                rst_InF;
    wire                                rst_InR;
    wire                                rst_tag;
    wire [2 : 0]                        inout_mode;
    wire                                key_A;
    wire                                key_B;
    wire                                key_C;
    wire                                key_F;
    wire [DATA_WIDTH - 1 : 0]           mask_A;
    wire [DATA_WIDTH - 1 : 0]           mask_B;
    wire [DATA_WIDTH - 1 : 0]           mask_R;
    wire                                mask_C;
    wire                                mask_F;
    wire [2 : 0]                        pass;

    wire [DATA_WIDTH - 1 : 0]           data_A_rbr;
    wire [DATA_DEPTH - 1 : 0]           data_A_cbc;
    wire [DATA_WIDTH - 1 : 0]           data_R_rbr;
    wire [DATA_DEPTH - 1 : 0]           data_R_cbc;
    wire [DATA_WIDTH - 1 : 0]           data_B_rbr;
    wire [DATA_DEPTH - 1 : 0]           data_B_cbc;
    wire [DATA_DEPTH - 1 : 0]           data_F;
    wire [DATA_DEPTH - 1 : 0]           data_C;

    wire [ADDR_WIDTH_MEM - 1 : 0]       addr_ins;
    wire                                ins_cache_rdy;
    wire [DDR_ADDR_WIDTH - 1 : 0]       jmp_addr_pc;

    wire [ISA_WIDTH - 1 : 0]            ins_to_apctrl;
    wire [OPCODE_WIDTH - 1 : 0]         ins_valid;
    wire                                ins_read_req;
    wire                                ddr_to_ic_rd_en;
    wire [DDR_ADDR_WIDTH -1 : 0]        ins_read_addr;
    wire [ISA_WIDTH - 1 : 0]            ins_to_cache;
    wire [7 : 0]                        rd_cnt_ins;
    wire                                rd_burst_data_valid;
    wire [7 : 0]                        ins_read_len;

    wire                                data_cache_rdy;
    wire                                jmp_addr_rdy;
    wire [DDR_ADDR_WIDTH - 1 : 0]		jmp_addr;
    wire [DATA_WIDTH - 1 : 0]           data_in_rbr;
    wire [DATA_DEPTH - 1 : 0]           data_in_cbc;
    wire [ADDR_WIDTH_MEM - 1 : 0]       addr_cur_ctxt;
    wire                                data_read_req;
    wire                                data_store_req;
    wire                                jmp_addr_read_req;
    wire [DDR_ADDR_WIDTH - 1 : 0]       jmp_addr_to_cache;
    wire [DATA_WIDTH - 1 : 0]           data_to_ddr;
	wire [DDR_ADDR_WIDTH - 1 : 0]		data_read_addr;
	wire [DDR_ADDR_WIDTH - 1 : 0]		data_write_addr;
    wire [DATA_WIDTH - 1 : 0]			data_to_cache;
    wire [9 : 0]                        rd_cnt_data;

    wire [ADDR_WIDTH_MEM - 1 : 0]       ret_addr_ret;
    wire [ADDR_WIDTH_MEM - 1 : 0]       ctxt_addr_ret;
    wire [DATA_WIDTH - 1 : 0]           tmp_bit_cnt_ret;
    wire [2 : 0]                        tmp_pass_ret;
    wire [DATA_WIDTH - 1 : 0]           tmp_mask_ret;
    wire [DATA_DEPTH - 1 : 0]           tmp_C_F_ret;
    wire                                ctxt_rdy;

    wire [29 : 0]                       ins_ddr_to_fifo;
    wire                                wr_en_ddr_to_ins_fifo;
    wire                                rd_en_ic_to_ins_fifo;
    wire [29 : 0]                       ins_fifo_to_cache;
    wire                                ddr_to_ic_full;
    wire                                ddr_to_ic_empty;
    wire                                ins_reading;
    wire [7 : 0]                        rd_data_cnt_to_ic;
    wire [7 : 0]                        wr_data_cnt_to_ddr_ins;
    wire                                prog_full;
    wire                                prog_empty;
    wire                                wr_rst_busy;
    wire                                rd_rst_busy;

    AP_controller #(
    DATA_WIDTH, 
    DATA_DEPTH, 
    OPCODE_WIDTH, 
    ADDR_WIDTH_CAM, 
    OPRAND_2_WIDTH,
    ADDR_WIDTH_MEM,
    DDR_ADDR_WIDTH
    )AP_controller_u 
    (
    .clk                    (sys_clk_i_1),
    .rst_STATE              (sys_rst),
    .rst_clk                (sys_rst),
    .int                    (int),
    .ins_valid              (ins_valid),
    .ins_to_apctrl          (ins_to_apctrl),
    .data_cache_rdy         (data_cache_rdy),
    .data_print             (data_print),
    .data_print_rdy         (data_print_rdy),
    .finish_flag            (finish_flag),
    .jmp_addr_rdy           (jmp_addr_rdy),
    .jmp_addr               (jmp_addr),
    .data_in_rbr            (data_in_rbr),
    .data_in_cbc            (data_in_cbc),
    .addr_cur_ctxt          (addr_cur_ctxt),
    .addr_cam_col           (addr_cam_col),
    .data_out_rbr           (data_out_rbr),
    .data_out_cbc           (data_out_cbc),
    .data_addr              (data_addr),
    .data_cmd               (data_cmd),
    .store_ddr_en           (store_ddr_en),
    .store_ctxt_finish      (store_ctxt_finish),
    .addr_ins               (addr_ins),
    .jmp_addr_pc            (jmp_addr_pc),
    .ins_inp_valid          (ins_inp_valid),
    .ret_addr_pc            (ret_addr_pc),
    .ret_addr_pc_rdy        (ret_addr_pc_rdy),
    .ret_addr_ret           (ret_addr_ret),
    .ctxt_addr_ret          (ctxt_addr_ret),
    .tmp_bit_cnt_ret        (tmp_bit_cnt_ret),
    .tmp_pass_ret           (tmp_pass_ret),
    .tmp_mask_ret           (tmp_mask_ret),
    .tmp_C_F_ret            (tmp_C_F_ret),
    .tmp_key_A_ret          (tmp_key_A_ret),
    .tmp_key_B_ret          (tmp_key_B_ret),
    .tmp_key_C_ret          (tmp_key_C_ret),
    .tmp_key_F_ret          (tmp_key_F_ret),
    .int_set                (int_set),
    .ret_valid              (ret_valid),
    .ret_addr               (ret_addr),
    .ctxt_addr              (ctxt_addr),
    .tmp_bit_cnt            (tmp_bit_cnt),
    .tmp_pass               (tmp_pass),
    .tmp_mask               (tmp_mask),
    .tmp_C_F                (tmp_C_F),
    .tmp_key_A              (tmp_key_A),
    .tmp_key_B              (tmp_key_B),
    .tmp_key_C              (tmp_key_C),
    .tmp_key_F              (tmp_key_F),
    .ctxt_rdy               (ctxt_rdy),
    .data_A_rbr             (data_A_rbr),
    .data_A_cbc             (data_A_cbc),
    .data_B_rbr             (data_B_rbr),
    .data_B_cbc             (data_B_cbc),
    .data_R_rbr             (data_R_rbr),
    .data_R_cbc             (data_R_cbc),
    .data_C                 (data_C),
    .data_F                 (data_F),
    .input_A_rbr            (input_A_rbr),
    .input_A_cbc            (input_A_cbc),
    .addr_input_rbr_A       (addr_input_rbr_A),
    .addr_input_cbc_A       (addr_input_cbc_A),
    .addr_output_rbr_A      (addr_output_rbr_A),
    .addr_output_cbc_A      (addr_output_cbc_A),
    .input_B_rbr            (input_B_rbr),
    .input_B_cbc            (input_B_cbc),
    .addr_input_rbr_B       (addr_input_rbr_B),
    .addr_input_cbc_B       (addr_input_cbc_B),
    .addr_output_rbr_B      (addr_output_rbr_B),
    .addr_output_cbc_B      (addr_output_cbc_B),
    .input_R_rbr            (input_R_rbr),
    .input_R_cbc            (input_R_cbc),
    .addr_input_rbr_R       (addr_input_rbr_R),
    .addr_input_cbc_R       (addr_input_cbc_R),
    .addr_output_rbr_R      (addr_output_rbr_R),
    .addr_output_cbc_R      (addr_output_cbc_R),
    .input_C                (input_C),
    .input_F                (input_F),
    .abs_opt                (abs_opt),
    .rst_InA                (rst_InA),
    .rst_InB                (rst_InB),
    .rst_InC                (rst_InC),
    .rst_InF                (rst_InF),
    .rst_InR                (rst_InR),
    .rst_tag                (rst_tag),
    .inout_mode             (inout_mode),
    .key_A                  (key_A),
    .key_B                  (key_B),
    .key_C                  (key_C),
    .key_F                  (key_F),
    .mask_A                 (mask_A),
    .mask_B                 (mask_B),
    .mask_R                 (mask_R),
    .mask_C                 (mask_C),
    .mask_F                 (mask_F),
    .pass                   (pass)
    );

    CAM #(
    DATA_WIDTH, 
    DATA_DEPTH,
    ADDR_WIDTH_CAM
    ) CAM_u 
    (
    .addr_input_rbr_A       (addr_input_rbr_A),
    .addr_input_cbc_A       (addr_input_cbc_A),
    .input_mode             (inout_mode),
    .rst_InA                (rst_InA),
    .addr_output_rbr_A      (addr_output_rbr_A),
    .addr_output_cbc_A      (addr_output_cbc_A),
    .input_A_row            (input_A_rbr),
    .input_A_col            (input_A_cbc),
    .key_A                  (key_A),
    .mask_A                 (mask_A),
    .mask_B                 (mask_B),
    .mask_R                 (mask_R),
    .clk                    (sys_clk_i_1),
    .addr_input_rbr_B       (addr_input_rbr_B),
    .addr_input_cbc_B       (addr_input_cbc_B),
    .rst_InB                (rst_InB),
    .addr_output_rbr_B      (addr_output_rbr_B),
    .addr_output_cbc_B      (addr_output_cbc_B),
    .input_B_row            (input_B_rbr),
    .input_B_col            (input_B_cbc),
    .key_B                  (key_B),
    .rst_InC                (rst_InC),
    .input_C                (input_C),
    .key_C                  (key_C),
    .mask_C                 (mask_C),
    .pass                   (pass),
    .rst_tag                (rst_tag),
    .rst_InF                (rst_InF),
    .input_F                (input_F),
    .key_F                  (key_F),
    .mask_F                 (mask_F),
    .addr_input_rbr_R       (addr_input_rbr_R),
    .addr_input_cbc_R       (addr_input_cbc_R),
    .rst_InR                (rst_InR),
    .addr_output_rbr_R      (addr_output_rbr_R),
    .addr_output_cbc_R      (addr_output_cbc_R),
    .input_R_row            (input_R_rbr),
    .input_R_col            (input_R_cbc),
    .abs_opt                (abs_opt),
    .Q_out_A_row            (data_A_rbr),
    .Q_out_A_col            (data_A_cbc),
    .Q_out_R_row            (data_R_rbr),
    .Q_out_R_col            (data_R_cbc),
    .Q_out_B_row            (data_B_rbr),
    .Q_out_B_col            (data_B_cbc),
    .Q_out_F                (data_F),
    .Q_out_C                (data_C)
    );

    ddr3_interface_top #(
    DDR_DATA_WIDTH, 
    DDR_ADDR_WIDTH, 
    ADDR_WIDTH_MEM,
    DATA_WIDTH,
    ISA_WIDTH,
    ISA_DEPTH,
    INT_INS_DEPTH,
    DATA_CACHE_DEPTH,
    TOTAL_ISA_DEPTH, 
    TOTAL_DATA_DEPTH
    ) ddr3_interface_top_u
    (
    .ddr3_dq                (ddr3_dq),
    .ddr3_dqs_n             (ddr3_dqs_n),
    .ddr3_dqs_p             (ddr3_dqs_p),
    .ddr3_addr              (ddr3_addr),
    .ddr3_ba                (ddr3_ba),
    .ddr3_ras_n             (ddr3_ras_n),
    .ddr3_cas_n             (ddr3_cas_n),
    .ddr3_we_n              (ddr3_we_n),
    .ddr3_reset_n           (ddr3_reset_n),
    .ddr3_ck_p              (ddr3_ck_p),
    .ddr3_ck_n              (ddr3_ck_n),
    .ddr3_cke               (ddr3_cke),
    .ddr3_cs_n              (ddr3_cs_n),
    .ddr3_dm                (ddr3_dm),
    .ddr3_odt               (ddr3_odt),
    .sys_clk_i              (sys_clk_i_2),
    .ui_clk                 (ui_clk),
    .init_calib_complete    (init_calib_complete),
    .sys_rst                (sys_rst),
    .ins_input              (ins_input),
    .data_input             (data_input),
    .wr_burst_data_req      (wr_burst_data_req),
    .load_ins_ddr           (load_ins_ddr),
    .load_data_ddr          (load_data_ddr),
    .load_int_ins_ddr       (load_int_ins_ddr),
    .ins_read_req           (ins_read_req),
    .ins_read_addr          (ins_read_addr),
    .ins_to_cache           (ins_ddr_to_fifo),
    .wr_en_ddr_to_ins_fifo  (wr_en_ddr_to_ins_fifo),
    .ddr_to_ic_empty        (ddr_to_ic_empty),
    .ins_reading            (ins_reading),
    .rd_burst_data_valid    (rd_burst_data_valid),
    .data_read_req          (data_read_req),
    .data_store_req         (data_store_req),
    .jmp_addr_read_req      (jmp_addr_read_req),
    .jmp_addr_to_cache      (jmp_addr_to_cache),
    .data_to_ddr            (data_to_ddr),
    .data_read_addr         (data_read_addr),
    .data_write_addr        (data_write_addr),
    .data_to_cache          (data_to_cache),
    .rd_cnt_data            (rd_cnt_data),
    .ins_read_len           (ins_read_len)
    );

    program_counter #(
    ADDR_WIDTH_MEM,
    ISA_DEPTH,
    TOTAL_ISA_DEPTH,
    DDR_ADDR_WIDTH
    ) program_counter_u 
    (
    .clk                    (sys_clk_i_1),
    .rst                    (sys_rst),
    .ret_valid              (ret_valid),
    .int                    (int),
    .ins_inp_valid          (ins_inp_valid),
    .ret_addr_pc            (ret_addr_pc),
    .ret_addr_pc_rdy        (ret_addr_pc_rdy),
    .jmp_addr_pc            (jmp_addr_pc),
    .addr_ins               (addr_ins),
    .ins_cache_rdy          (ins_cache_rdy)
    );

    fifo_generator_0 fifo_ins_to_cache_u
    (
    .rst                    (!sys_rst), // input wire rst
    .wr_clk                 (sys_clk_i_2), // input wire wr_clk
    .rd_clk                 (sys_clk_i_1), // input wire rd_clk
    .din                    (ins_ddr_to_fifo), // input wire [29 : 0] din
    .wr_en                  (wr_en_ddr_to_ins_fifo), // input wire wr_en
    .rd_en                  (ddr_to_ic_rd_en), // input wire rd_en
    .dout                   (ins_fifo_to_cache), // output wire [29 : 0] dout
    .full                   (ddr_to_ic_full), // output wire full
    .empty                  (ddr_to_ic_empty), // output wire empty
    .rd_data_count          (rd_data_cnt_to_ic), // output wire [7 : 0] rd_data_count
    .wr_data_count          (wr_data_cnt_to_ddr_ins), // output wire [7 : 0] wr_data_count
    .prog_full              (prog_full), // output wire prog_full
    .prog_empty             (prog_empty), // output wire prog_empty
    .wr_rst_busy            (wr_rst_busy), // output wire wr_rst_busy
    .rd_rst_busy            (rd_rst_busy) // output wire rd_rst_busy
    );

    ins_cache #(
    ISA_DEPTH,
    INT_INS_DEPTH,
    DDR_ADDR_WIDTH,
    OPCODE_WIDTH, 
    ADDR_WIDTH_CAM, 
    OPRAND_2_WIDTH,
    ADDR_WIDTH_MEM,
    TOTAL_ISA_DEPTH
    )ins_cache_u 
    (
    .clk                    (sys_clk_i_1),
    .rst                    (sys_rst),
    .addr_ins               (addr_ins),
    .ins_cache_rdy          (ins_cache_rdy),
    .ins_to_apctrl          (ins_to_apctrl),
    .ins_valid              (ins_valid),
    .ins_read_req           (ins_read_req),
    .ins_reading            (ins_reading),
    .ddr_to_ic_rd_en        (ddr_to_ic_rd_en),
    .ins_read_addr          (ins_read_addr),
    .ins_to_cache           (ins_fifo_to_cache),
    //.rd_cnt_ins             (rd_data_cnt_to_ic),
    .rd_burst_data_valid    (prog_full),
    .ddr_to_ic_empty        (ddr_to_ic_empty),
    .ins_read_len           (ins_read_len)
    );

    
    data_cache #(
    DATA_CACHE_DEPTH,
    DATA_WIDTH,
    DATA_DEPTH,
    DDR_ADDR_WIDTH,
    ADDR_WIDTH_MEM,
    ADDR_WIDTH_CAM
    )data_cache_u 
    (
    .clk                    (sys_clk_i_1),
    .rst                    (sys_rst),
    .int_set                (int_set),
    .data_out_rbr           (data_out_rbr),
    .data_out_cbc           (data_out_cbc),
    .data_addr              (data_addr),
    .data_cmd               (data_cmd),
    .addr_cam_col           (addr_cam_col),
    .store_ddr_en           (store_ddr_en),
    .store_ctxt_finish      (store_ctxt_finish),
    .data_cache_rdy         (data_cache_rdy),
    .jmp_addr_rdy           (jmp_addr_rdy),
    .jmp_addr               (jmp_addr),
    .data_in_rbr            (data_in_rbr),
    .data_in_cbc            (data_in_cbc),
    .addr_cur_ctxt          (addr_cur_ctxt),
    .data_read_req          (data_read_req),
    .data_store_req         (data_store_req),
    .jmp_addr_read_req      (jmp_addr_read_req),
    .jmp_addr_to_cache      (jmp_addr_to_cache),
    .data_to_ddr            (data_to_ddr),
    .data_read_addr         (data_read_addr),
    .data_write_addr        (data_write_addr),
    .data_to_cache          (data_to_cache),
    .rd_cnt_data            (rd_cnt_data),
    .rd_burst_data_valid    (rd_burst_data_valid)
    );

    
    int_stack #(
    DATA_WIDTH,
    DATA_DEPTH,
    ADDR_WIDTH_MEM
    )int_stack_u (
    .clk                    (sys_clk_i_1),
    .rst                    (sys_rst),
    .int_set                (int_set),
    .ret_valid              (ret_valid),
    .ret_addr               (ret_addr),
    .ctxt_addr              (ctxt_addr),
    .tmp_bit_cnt            (tmp_bit_cnt),
    .tmp_pass               (tmp_pass),
    .tmp_mask               (tmp_mask),
    .tmp_C_F                (tmp_C_F),
    .tmp_key_A              (tmp_key_A),
    .tmp_key_B              (tmp_key_B),
    .tmp_key_C              (tmp_key_C),
    .tmp_key_F              (tmp_key_F),
    .ctxt_rdy               (ctxt_rdy),
    .ret_addr_ret           (ret_addr_ret),
    .ctxt_addr_ret          (ctxt_addr_ret),
    .tmp_bit_cnt_ret        (tmp_bit_cnt_ret),
    .tmp_pass_ret           (tmp_pass_ret),
    .tmp_mask_ret           (tmp_mask_ret),
    .tmp_C_F_ret            (tmp_C_F_ret),
    .tmp_key_A_ret          (tmp_key_A_ret),
    .tmp_key_B_ret          (tmp_key_B_ret),
    .tmp_key_C_ret          (tmp_key_C_ret),
    .tmp_key_F_ret          (tmp_key_F_ret)
    );
    
endmodule
