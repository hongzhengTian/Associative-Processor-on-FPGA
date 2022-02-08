module ddr3_interface_top
#(
    parameter DDR_DATA_WIDTH    = 128,
    parameter DDR_ADDR_WIDTH    = 28,
    parameter ADDR_WIDTH_MEM	= 16,
    parameter DATA_WIDTH        = 16,
    parameter ISA_WIDTH         = 30,
    parameter ISA_DEPTH         = 72,
    parameter INT_INS_DEPTH     = 27,
    parameter DATA_CACHE_DEPTH  = 16,
    parameter TOTAL_ISA_DEPTH   = 128,
    parameter TOTAL_DATA_DEPTH  = 128
)
    (
    /* interface of system */
    input                               sys_clk_i,
    output                              init_calib_complete,
    input [ISA_WIDTH - 1 : 0]           ins_input,
    input [DATA_WIDTH - 1 : 0]          data_input,
    input			                    sys_rst,
    output                              wr_burst_data_req, 
    output                              load_ins_ddr,
	output                              load_data_ddr,
	output                              load_int_ins_ddr,
    output                              ui_clk,

    /* interface of ISA_cache */
    input                               ins_read_req,
	input  [DDR_ADDR_WIDTH - 1 : 0]		ins_read_addr,
    output [ISA_WIDTH - 1 : 0]			ins_to_cache,
    output [9 : 0]                      rd_cnt_isa,
    output                              rd_burst_data_valid,


    /* interface of DATA_cache */
    input                               data_read_req,
    input                               data_store_req,
    input                               jmp_addr_read_req,
    output [DDR_ADDR_WIDTH - 1 : 0]	    jmp_addr_to_cache,
    input  [DATA_WIDTH - 1 : 0]         data_to_ddr,
	input  [DDR_ADDR_WIDTH - 1 : 0]		data_read_addr,
	input  [DDR_ADDR_WIDTH - 1 : 0]		data_write_addr,
    output [DATA_WIDTH - 1 : 0]			data_to_cache, 
    output [9 : 0] 						rd_cnt_data,
    input  [9 : 0]                      ins_read_len,
    
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

    wire                        app_rdy             ;
    wire                        app_wdf_rdy         ;
    wire                        app_en              ;
    wire [27:0]                 app_addr            ;
    wire [ 2:0]                 app_cmd             ;
    wire [15:0]                 app_wdf_mask        ;
    wire                        app_wdf_wren        ;
    wire [127:0]                app_wdf_data        ;
    wire                        app_wdf_end         ;
    wire [127:0]                app_rd_data         ;
    wire                        app_rd_data_valid   ;
    wire                        app_rd_data_end     ;
    wire                        ui_clk_sync_rst     ;

    wire                        rd_burst_req;                       
	wire                        wr_burst_req;                  
	wire [9:0]                  rd_burst_len;                 
	wire [9:0]                  wr_burst_len;                    
	wire [DDR_ADDR_WIDTH - 1:0] rd_burst_addr;    
	wire [DDR_ADDR_WIDTH - 1:0] wr_burst_addr;      
	           
    wire [9:0]                  rd_addr_cnt;
	wire [DDR_DATA_WIDTH - 1:0] rd_burst_data; 
	wire [DDR_DATA_WIDTH - 1:0] wr_burst_data;  
	wire                        rd_burst_finish;              
	wire                        wr_burst_finish;           
	wire                        burst_finish;
    wire [11:0]                 device_temp;
    
    DDR_cache_interface #(
    DDR_DATA_WIDTH, 
    DDR_ADDR_WIDTH, 
    ADDR_WIDTH_MEM,
    DATA_WIDTH,
    ISA_WIDTH,
    ISA_DEPTH,
    DATA_CACHE_DEPTH,
    TOTAL_ISA_DEPTH, 
    TOTAL_DATA_DEPTH,
    INT_INS_DEPTH
    ) u_DDR_cache_interface
    (
        .mem_clk(ui_clk),
        .rst(ui_clk_sync_rst),
        .ins_input(ins_input),
        .data_input(data_input),
        .ins_read_req(ins_read_req),
        .ins_read_addr(ins_read_addr),
        .ins_to_cache(ins_to_cache),
        .rd_cnt_isa(rd_cnt_isa),
        .data_read_req(data_read_req),
        .data_store_req(data_store_req),
        .jmp_addr_read_req(jmp_addr_read_req),
        .data_to_ddr(data_to_ddr),
        .data_read_addr(data_read_addr),
        .data_write_addr(data_write_addr),
        .data_to_cache(data_to_cache),
        .rd_cnt_data(rd_cnt_data),
        .rd_burst_req(rd_burst_req),
        .wr_burst_req(wr_burst_req),
        .rd_burst_len(rd_burst_len),
        .wr_burst_len(wr_burst_len),
        .rd_burst_addr(rd_burst_addr),
        .wr_burst_addr(wr_burst_addr),
        .rd_burst_data_valid(rd_burst_data_valid),
        .wr_burst_data_req(wr_burst_data_req),
        .jmp_addr_to_cache(jmp_addr_to_cache),
        .rd_burst_data(rd_burst_data),
        .wr_burst_data(wr_burst_data),
        .rd_burst_finish(rd_burst_finish),
        .wr_burst_finish(wr_burst_finish),
        .load_ins_ddr(load_ins_ddr),
        .load_data_ddr(load_data_ddr),
        .load_int_ins_ddr(load_int_ins_ddr),
        .ins_read_len(ins_read_len)
    );

    ddr_controller #(
    DDR_DATA_WIDTH, 
    DDR_ADDR_WIDTH
    )u_ddr_controller 
    (
        .clk(ui_clk),
        .rst(ui_clk_sync_rst),
        .rd_burst_req(rd_burst_req),
        .wr_burst_req(wr_burst_req),
        .rd_burst_len(rd_burst_len),
        .wr_burst_len(wr_burst_len),
        .rd_burst_addr(rd_burst_addr),
        .wr_burst_addr(wr_burst_addr),
        .rd_burst_data_valid(rd_burst_data_valid),
        .wr_burst_data_req(wr_burst_data_req),
        .rd_burst_data(rd_burst_data),
        .wr_burst_data(wr_burst_data),
        .rd_burst_finish(rd_burst_finish),
        .wr_burst_finish(wr_burst_finish),
        .burst_finish(burst_finish),
        .rd_addr_cnt(rd_addr_cnt),
        .app_rd_data(app_rd_data),
        .app_rd_data_valid(app_rd_data_valid),
        .app_rdy(app_rdy),
        .app_wdf_rdy(app_wdf_rdy),
        .init_calib_complete(init_calib_complete),
        .app_addr(app_addr),
        .app_cmd(app_cmd),
        .app_wdf_data(app_wdf_data),
        .app_wdf_end(app_wdf_end),
        .app_wdf_wren(app_wdf_wren),
        .app_en(app_en),
        .app_wdf_mask(app_wdf_mask)
    );

    mig_7series_0 u_mig_7series_0 
    (
    .ddr3_addr                      (ddr3_addr),  // output [13:0]		ddr3_addr
    .ddr3_ba                        (ddr3_ba),  // output [2:0]		ddr3_ba
    .ddr3_cas_n                     (ddr3_cas_n),  // output			ddr3_cas_n
    .ddr3_ck_n                      (ddr3_ck_n),  // output [0:0]		ddr3_ck_n
    .ddr3_ck_p                      (ddr3_ck_p),  // output [0:0]		ddr3_ck_p
    .ddr3_cke                       (ddr3_cke),  // output [0:0]		ddr3_cke
    .ddr3_ras_n                     (ddr3_ras_n),  // output			ddr3_ras_n
    .ddr3_reset_n                   (ddr3_reset_n),  // output			ddr3_reset_n
    .ddr3_we_n                      (ddr3_we_n),  // output			ddr3_we_n
    .ddr3_dq                        (ddr3_dq),  // inout [15:0]		ddr3_dq
    .ddr3_dqs_n                     (ddr3_dqs_n),  // inout [1:0]		ddr3_dqs_n
    .ddr3_dqs_p                     (ddr3_dqs_p),  // inout [1:0]		ddr3_dqs_p
    .init_calib_complete            (init_calib_complete),  // output			init_calib_complete
	.ddr3_cs_n                      (ddr3_cs_n),  // output [0:0]		ddr3_cs_n
    .ddr3_dm                        (ddr3_dm),  // output [1:0]		ddr3_dm
    .ddr3_odt                       (ddr3_odt),  // output [0:0]		ddr3_odt
    // Application interface ports
    .app_addr                       (app_addr),  // input [27:0]		app_addr
    .app_cmd                        (app_cmd),  // input [2:0]		app_cmd
    .app_en                         (app_en),  // input				app_en
    .app_wdf_data                   (app_wdf_data),  // input [127:0]		app_wdf_data
    .app_wdf_end                    (app_wdf_end),  // input				app_wdf_end
    .app_wdf_wren                   (app_wdf_wren),  // input				app_wdf_wren
    .app_rd_data                    (app_rd_data),  // output [127:0]		app_rd_data
    .app_rd_data_end                (),  // output			app_rd_data_end
    .app_rd_data_valid              (app_rd_data_valid),  // output			app_rd_data_valid
    .app_rdy                        (app_rdy),  // output			app_rdy
    .app_wdf_rdy                    (app_wdf_rdy),  // output			app_wdf_rdy
    .app_sr_req                     (1'b0),  // input			app_sr_req
    .app_ref_req                    (1'b0),  // input			app_ref_req
    .app_zq_req                     (1'b0),  // input			app_zq_req
    .app_sr_active                  (),  // output			app_sr_active
    .app_ref_ack                    (),  // output			app_ref_ack
    .app_zq_ack                     (),  // output			app_zq_ack
    .ui_clk                         (ui_clk),  // output			ui_clk
    .ui_clk_sync_rst                (ui_clk_sync_rst),  // output			ui_clk_sync_rst
    .app_wdf_mask                   (app_wdf_mask),  // input [15:0]		app_wdf_mask
    .device_temp                    (device_temp),
    // System Clock Ports
    .sys_clk_i                      (sys_clk_i),
    .sys_rst                        (sys_rst) // input sys_rst
    );
endmodule
