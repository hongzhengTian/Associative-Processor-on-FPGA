`timescale 1ps/100fs
module sim_tb_top;
// The following parameters refer to width of various ports
    parameter                       COL_WIDTH               = 10;// # of memory Column Address bits.
    parameter                       CS_WIDTH                = 1;// # of unique CS outputs to memory.
    parameter                       DM_WIDTH                = 2;// # of DM (data mask)
    parameter                       DQ_WIDTH                = 16;// # of DQ (data)
    parameter                       DQS_WIDTH               = 2;
    parameter                       DQS_CNT_WIDTH           = 1;// = ceil(log2(DQS_WIDTH))
    parameter                       DRAM_WIDTH              = 8;// # of DQ per DQS
    parameter                       ECC                     = "OFF";
    parameter                       RANKS                   = 1;// # of Ranks.
    parameter                       ODT_WIDTH               = 1;// # of ODT outputs to memory.
    parameter                       ROW_WIDTH               = 14;// # of memory Row Address bits.
    parameter                       ADDR_WIDTH              = 28;

    parameter                       DDR_DATA_WIDTH          = 128;
	parameter                       DDR_ADDR_WIDTH          = 28;
    parameter                       DATA_WIDTH              = 8;
    parameter                       DATA_DEPTH              = 16;
    parameter                       ISA_DEPTH               = 128;
    parameter                       DATA_CACHE_DEPTH        = 16;
    parameter                       OPCODE_WIDTH            = 4;
    parameter                       ADDR_WIDTH_CAM          = 8;
    parameter                       OPRAND_2_WIDTH          = 2;
    parameter                       ADDR_WIDTH_MEM          = 16;
    parameter                       ISA_WIDTH               = OPCODE_WIDTH 
                                                            + ADDR_WIDTH_CAM
                                                            + OPRAND_2_WIDTH 
                                                            + ADDR_WIDTH_MEM; 

    localparam                      TOTAL_ISA_DEPTH         = 234;
    localparam                      CACHE_ISA_ADDR          = 10;
    localparam                      CACHE_DATA_ADDR         = 10;
    localparam                      TOTAL_DATA_DEPTH        = 128;
    localparam                      INT_INS_DEPTH           = 28;
    
    //The following parameters are mode register settings
    parameter                       BURST_MODE              = "8";// DDR3 SDRAM:Burst Length (Mode Register 0).
                                                                  // # = "8", "4", "OTF".
    parameter                       CA_MIRROR               = "OFF";// C/A mirror opt for DDR3 dual rank

    // The following parameters are multiplier and divisor factors for PLLE2.
    // Based on the selected design frequency these parameters vary.
    parameter                       CLKIN_PERIOD_1          = 5000;// Input Clock Period
    parameter                       CLKIN_PERIOD_2          = 5000;// Input Clock Period

    parameter                       CLKIN_CAM_PERIOD        = 5000;//CLKIN_PERIOD / 8;

    // Simulation parameters
    parameter                       SIM_BYPASS_INIT_CAL     = "FAST";// # = "SIM_INIT_CAL_FULL" -  Complete
                                        //              memory init &
                                        //              calibration sequence
                                        // # = "SKIP" - Not supported
                                        // # = "FAST" - Complete memory init & use
                                        //              abbreviated calib sequence

    // IODELAY and PHY related parameters
    parameter                       RST_ACT_LOW             = 1;// =1 for active low reset,=0 for active high.
    // Referece clock frequency parameters
    parameter                       REFCLK_FREQ             = 200.0;// IODELAYCTRL reference clock frequency

    // Local parameters Declarations
    localparam real                 TPROP_DQS               = 0.00;// Delay for DQS signal during Write Operation
    localparam real                 TPROP_DQS_RD            = 0.00;// Delay for DQS signal during Read Operation
    localparam real                 TPROP_PCB_CTRL          = 0.00;// Delay for Address and Ctrl signals
    localparam real                 TPROP_PCB_DATA          = 0.00;// Delay for data signal during Write operation
    localparam real                 TPROP_PCB_DATA_RD       = 0.00;// Delay for data signal during Read operation

    localparam                      MEMORY_WIDTH            = 8;
    localparam                      NUM_COMP                = DQ_WIDTH/MEMORY_WIDTH;
    localparam                      ECC_TEST 		   	    = "OFF" ;
    localparam                      ERR_INSERT              = (ECC_TEST == "ON") ? "OFF" : ECC ;

    localparam real                 REFCLK_PERIOD           = (1000000.0/(2*REFCLK_FREQ));
    localparam                      RESET_PERIOD            = 200000; //in pSec  

    // Wire Declarations
    reg                                 sys_rst_n;
    wire                                sys_rst;
    reg                                 sys_clk_i_1;
    reg                                 sys_clk_i_2;
    reg                                 clk_ref_i;
    wire                                ddr3_reset_n;
    wire [DQ_WIDTH-1:0]                 ddr3_dq_fpga;
    wire [DQS_WIDTH-1:0]                ddr3_dqs_p_fpga;
    wire [DQS_WIDTH-1:0]                ddr3_dqs_n_fpga;
    wire [ROW_WIDTH-1:0]                ddr3_addr_fpga;
    wire [3-1:0]                        ddr3_ba_fpga;
    wire                                ddr3_ras_n_fpga;
    wire                                ddr3_cas_n_fpga;
    wire                                ddr3_we_n_fpga;
    wire [1-1:0]                        ddr3_cke_fpga;
    wire [1-1:0]                        ddr3_ck_p_fpga;
    wire [1-1:0]                        ddr3_ck_n_fpga;
    wire                                init_calib_complete;
    wire                                tg_compare_error;
    wire [(CS_WIDTH*1)-1:0]             ddr3_cs_n_fpga;
    wire [DM_WIDTH-1:0]                 ddr3_dm_fpga;
    wire [ODT_WIDTH-1:0]                ddr3_odt_fpga;
    reg [(CS_WIDTH*1)-1:0]              ddr3_cs_n_sdram_tmp;
    reg [DM_WIDTH-1:0]                  ddr3_dm_sdram_tmp;
    reg [ODT_WIDTH-1:0]                 ddr3_odt_sdram_tmp;
    wire [DQ_WIDTH-1:0]                 ddr3_dq_sdram;
    reg [ROW_WIDTH-1:0]                 ddr3_addr_sdram [0:1];
    reg [3-1:0]                         ddr3_ba_sdram [0:1];
    reg                                 ddr3_ras_n_sdram;
    reg                                 ddr3_cas_n_sdram;
    reg                                 ddr3_we_n_sdram;
    wire [(CS_WIDTH*1)-1:0]             ddr3_cs_n_sdram;
    wire [ODT_WIDTH-1:0]                ddr3_odt_sdram;
    reg [1-1:0]                         ddr3_cke_sdram;
    wire [DM_WIDTH-1:0]                 ddr3_dm_sdram;
    wire [DQS_WIDTH-1:0]                ddr3_dqs_p_sdram;
    wire [DQS_WIDTH-1:0]                ddr3_dqs_n_sdram;
    reg [1-1:0]                         ddr3_ck_p_sdram;
    reg [1-1:0]                         ddr3_ck_n_sdram;
    wire                                app_rdy;
    wire                                load_ins_ddr;
	wire                                load_data_ddr;
	wire                                load_int_ins_ddr;
    wire                                wr_burst_data_req;

    wire [DATA_WIDTH - 1 : 0]           data_print;
    wire                                data_print_rdy;
    wire                                finish_flag;
    reg                                 int;


    reg [ISA_WIDTH - 1 : 0]             MEM_ISA     [0 : TOTAL_ISA_DEPTH - 1];
    reg [DATA_WIDTH - 1 : 0]            MEM_DATA    [0 : TOTAL_DATA_DEPTH - 1];
    reg [ISA_WIDTH - 1 : 0]             MEM_INT_INS [0 : INT_INS_DEPTH - 1];
    reg [DATA_WIDTH - 1 : 0]            MEM_DATA_REF[0 : TOTAL_DATA_DEPTH - 1];
    reg [CACHE_ISA_ADDR - 1 : 0]        MEM_ADDR    = 0;
    reg [CACHE_ISA_ADDR - 1 : 0]        MEM_ADDR_INT = 0;
    reg [CACHE_DATA_ADDR - 1 : 0]       MEM_ADDR_DATA = 0;
    reg [CACHE_DATA_ADDR - 1 : 0]       MEM_ADDR_DATA_REF = 0;
    reg [ISA_WIDTH - 1 : 0]             instruction_reg;
    wire [ISA_WIDTH - 1 : 0]            instruction;
    reg [DATA_WIDTH - 1 : 0]            data_reg;
    wire [DATA_WIDTH -1 : 0]            data;

    integer                             outputfile;
    integer                             cnt_wrong;

    initial 
    begin
        $readmemb("ISA_Bin.txt", MEM_ISA);
        $readmemb("DATA.txt", MEM_DATA);
        $readmemb("ISA_interrupt_Bin.txt", MEM_INT_INS);
        $readmemb("data_ref.txt", MEM_DATA_REF);
        outputfile = $fopen("AP_output.txt", "w");
        wait (finish_flag);
        $display("total wrong line : total test number =%d:%d\n", cnt_wrong, MEM_ADDR_DATA_REF + 1);
        if (cnt_wrong == 0)
            begin
                $display("simulation success\n");
            end
        $fclose(outputfile);
        $finish;
    end

    always @(posedge sys_clk_i_2) 
    begin
        if(wr_burst_data_req & load_ins_ddr)
        begin
          instruction_reg = MEM_ISA[MEM_ADDR];
          MEM_ADDR = MEM_ADDR + 1;
        end

        else if(wr_burst_data_req & load_data_ddr)
        begin
          data_reg = MEM_DATA[MEM_ADDR_DATA];
          MEM_ADDR_DATA = MEM_ADDR_DATA + 1;
        end

        else if(wr_burst_data_req & load_int_ins_ddr)
        begin
          instruction_reg = MEM_INT_INS[MEM_ADDR_INT];
          MEM_ADDR_INT = MEM_ADDR_INT + 1;
        end
    end

    always @(posedge data_print_rdy)
    begin
        if(data_print_rdy)
        begin
            $fwrite(outputfile, "%b\n", data_print);
            MEM_ADDR_DATA_REF <= MEM_ADDR_DATA_REF + 1;
                if (data_print != MEM_DATA_REF[MEM_ADDR_DATA_REF])
                    begin
                        $display("this line is wrong: %d\n", MEM_ADDR_DATA_REF + 1);
                        $display("result: %d\n", data_print);
                        $display("answer: %d\n", MEM_DATA_REF[MEM_ADDR_DATA_REF]);
                        cnt_wrong <= cnt_wrong + 1;
                    end
        end
    end

    assign instruction = instruction_reg;
    assign data = data_reg;

    initial begin
        int = 0;
        cnt_wrong = 0;
        #58977900
        int = 1;
        #6000
        int = 0;
    end

    // Reset Generation
    initial begin
    sys_rst_n = 1'b0;
    #RESET_PERIOD
    sys_rst_n = 1'b1;
    end
    assign sys_rst = RST_ACT_LOW ? sys_rst_n : ~sys_rst_n;

    // Clock Generation
    initial
        sys_clk_i_1 = 1'b0;
    always
        sys_clk_i_1 = #(CLKIN_PERIOD_1/2.0) ~sys_clk_i_1;
    initial
        sys_clk_i_2 = 1'b0;
    always
        sys_clk_i_2 = #(CLKIN_PERIOD_2/2.0) ~sys_clk_i_2;

    initial
        clk_ref_i = 1'b0;
    always
        clk_ref_i = #REFCLK_PERIOD ~clk_ref_i;


    always @( * ) begin
        ddr3_ck_p_sdram      <=  #(TPROP_PCB_CTRL) ddr3_ck_p_fpga;
        ddr3_ck_n_sdram      <=  #(TPROP_PCB_CTRL) ddr3_ck_n_fpga;
        ddr3_addr_sdram[0]   <=  #(TPROP_PCB_CTRL) ddr3_addr_fpga;
        ddr3_addr_sdram[1]   <=  #(TPROP_PCB_CTRL) (CA_MIRROR == "ON") ?
                                                    {ddr3_addr_fpga[ROW_WIDTH-1:9],
                                                    ddr3_addr_fpga[7], ddr3_addr_fpga[8],
                                                    ddr3_addr_fpga[5], ddr3_addr_fpga[6],
                                                    ddr3_addr_fpga[3], ddr3_addr_fpga[4],
                                                    ddr3_addr_fpga[2:0]} :
                                                    ddr3_addr_fpga;
        ddr3_ba_sdram[0]     <=  #(TPROP_PCB_CTRL) ddr3_ba_fpga;
        ddr3_ba_sdram[1]     <=  #(TPROP_PCB_CTRL) (CA_MIRROR == "ON") ?
                                                    {ddr3_ba_fpga[3-1:2],
                                                    ddr3_ba_fpga[0],
                                                    ddr3_ba_fpga[1]} :
                                                    ddr3_ba_fpga;
        ddr3_ras_n_sdram     <=  #(TPROP_PCB_CTRL) ddr3_ras_n_fpga;
        ddr3_cas_n_sdram     <=  #(TPROP_PCB_CTRL) ddr3_cas_n_fpga;
        ddr3_we_n_sdram      <=  #(TPROP_PCB_CTRL) ddr3_we_n_fpga;
        ddr3_cke_sdram       <=  #(TPROP_PCB_CTRL) ddr3_cke_fpga;
    end

    always @( * )
        ddr3_cs_n_sdram_tmp   <=  #(TPROP_PCB_CTRL) ddr3_cs_n_fpga;
    assign ddr3_cs_n_sdram =  ddr3_cs_n_sdram_tmp;

    always @( * )
        ddr3_dm_sdram_tmp <=  #(TPROP_PCB_DATA) ddr3_dm_fpga;//DM signal generation
    assign ddr3_dm_sdram = ddr3_dm_sdram_tmp;

    always @( * )
        ddr3_odt_sdram_tmp  <=  #(TPROP_PCB_CTRL) ddr3_odt_fpga;
    assign ddr3_odt_sdram =  ddr3_odt_sdram_tmp;

    // Controlling the bi-directional BUS
    genvar dqwd;
    generate
        for (dqwd = 1;dqwd < DQ_WIDTH;dqwd = dqwd+1) begin : dq_delay
        WireDelay #
        (
            .Delay_g    (TPROP_PCB_DATA),
            .Delay_rd   (TPROP_PCB_DATA_RD),
            .ERR_INSERT ("OFF")
        )
        u_delay_dq
        (
            .A             (ddr3_dq_fpga[dqwd]),
            .B             (ddr3_dq_sdram[dqwd]),
            .reset         (sys_rst_n),
            .phy_init_done (init_calib_complete)
        );
        end
            WireDelay #
        (
            .Delay_g    (TPROP_PCB_DATA),
            .Delay_rd   (TPROP_PCB_DATA_RD),
            .ERR_INSERT ("OFF")
        )
        u_delay_dq_0
        (
            .A             (ddr3_dq_fpga[0]),
            .B             (ddr3_dq_sdram[0]),
            .reset         (sys_rst_n),
            .phy_init_done (init_calib_complete)
        );
    endgenerate

    genvar dqswd;
    generate
        for (dqswd = 0;dqswd < DQS_WIDTH;dqswd = dqswd+1) begin : dqs_delay
        WireDelay #
        (
            .Delay_g    (TPROP_DQS),
            .Delay_rd   (TPROP_DQS_RD),
            .ERR_INSERT ("OFF")
        )
        u_delay_dqs_p
        (
            .A             (ddr3_dqs_p_fpga[dqswd]),
            .B             (ddr3_dqs_p_sdram[dqswd]),
            .reset         (sys_rst_n),
            .phy_init_done (init_calib_complete)
        );

        WireDelay #
        (
            .Delay_g    (TPROP_DQS),
            .Delay_rd   (TPROP_DQS_RD),
            .ERR_INSERT ("OFF")
        )
        u_delay_dqs_n
        (
            .A             (ddr3_dqs_n_fpga[dqswd]),
            .B             (ddr3_dqs_n_sdram[dqswd]),
            .reset         (sys_rst_n),
            .phy_init_done (init_calib_complete)
        );
        end
    endgenerate

    // FPGA Memory Controller
    AP_top #(
     DATA_WIDTH,
     DATA_DEPTH,
     ISA_DEPTH,
     INT_INS_DEPTH,
     TOTAL_ISA_DEPTH,
     TOTAL_DATA_DEPTH,
     DATA_CACHE_DEPTH,
     DDR_ADDR_WIDTH,
     DDR_DATA_WIDTH,
     OPCODE_WIDTH,
     ADDR_WIDTH_CAM,
     OPRAND_2_WIDTH,
     ADDR_WIDTH_MEM
      ) u_AP_top
        (
        .sys_clk_i_1            (sys_clk_i_1),
        .sys_clk_i_2            (sys_clk_i_2),
        .int                    (int),
        .init_calib_complete    (init_calib_complete),
        .sys_rst                (sys_rst),
        .ins_input              (instruction),
        .data_input             (data),
        .wr_burst_data_req      (wr_burst_data_req),
        .load_ins_ddr           (load_ins_ddr),
        .load_data_ddr          (load_data_ddr),
        .load_int_ins_ddr       (load_int_ins_ddr),
        .data_print             (data_print),
        .data_print_rdy         (data_print_rdy),
        .finish_flag            (finish_flag),

        .ddr3_dq                (ddr3_dq_fpga),
        .ddr3_dqs_n             (ddr3_dqs_n_fpga),
        .ddr3_dqs_p             (ddr3_dqs_p_fpga),
        .ddr3_addr              (ddr3_addr_fpga),
        .ddr3_ba                (ddr3_ba_fpga),
        .ddr3_ras_n             (ddr3_ras_n_fpga),
        .ddr3_cas_n             (ddr3_cas_n_fpga),
        .ddr3_we_n              (ddr3_we_n_fpga),
        .ddr3_reset_n           (ddr3_reset_n),
        .ddr3_ck_p              (ddr3_ck_p_fpga),
        .ddr3_ck_n              (ddr3_ck_n_fpga),
        .ddr3_cke               (ddr3_cke_fpga),
        .ddr3_cs_n              (ddr3_cs_n_fpga),
        .ddr3_dm                (ddr3_dm_fpga),
        .ddr3_odt               (ddr3_odt_fpga)
        );

    // Memory Models instantiations
    genvar r,i;
    generate
        for (r = 0; r < CS_WIDTH; r = r + 1) begin: mem_rnk
        for (i = 0; i < NUM_COMP; i = i + 1) begin: gen_mem
            ddr3_model u_comp_ddr3
            (
            .rst_n   (ddr3_reset_n),
            .ck      (ddr3_ck_p_sdram),
            .ck_n    (ddr3_ck_n_sdram),
            .cke     (ddr3_cke_sdram[r]),
            .cs_n    (ddr3_cs_n_sdram[r]),
            .ras_n   (ddr3_ras_n_sdram),
            .cas_n   (ddr3_cas_n_sdram),
            .we_n    (ddr3_we_n_sdram),
            .dm_tdqs (ddr3_dm_sdram[i]),
            .ba      (ddr3_ba_sdram[r]),
            .addr    (ddr3_addr_sdram[r]),
            .dq      (ddr3_dq_sdram[MEMORY_WIDTH*(i+1)-1:MEMORY_WIDTH*(i)]),
            .dqs     (ddr3_dqs_p_sdram[i]),
            .dqs_n   (ddr3_dqs_n_sdram[i]),
            .tdqs_n  (),
            .odt     (ddr3_odt_sdram[r])
            );
        end
        end
    endgenerate

    

endmodule