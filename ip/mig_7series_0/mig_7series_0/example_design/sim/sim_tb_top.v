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
    parameter                       ISA_DEPTH               = 132;
    parameter                       DATA_CACHE_DEPTH        = 16;
    parameter                       OPCODE_WIDTH            = 4;
    parameter                       ADDR_WIDTH_CAM          = 8;
    parameter                       OPRAND_2_WIDTH          = 2;
    parameter                       ADDR_WIDTH_MEM          = 16;
    parameter                       ISA_WIDTH               = OPCODE_WIDTH 
                                                            + ADDR_WIDTH_CAM
                                                            + OPRAND_2_WIDTH 
                                                            + ADDR_WIDTH_MEM; 
    
    localparam                      MEM_WRITE_ISA           = 4'd1;
    localparam                      MEM_WRITE_DATA          = 4'd3;

    localparam                      TOTAL_ISA_DEPTH         = 132;
    localparam                      CACHE_ISA_ADDR          = 10;
    localparam                      CACHE_DATA_ADDR         = 10;
    localparam                      TOTAL_DATA_DEPTH        = 128;
    
    //The following parameters are mode register settings
    parameter                       BURST_MODE              = "8";// DDR3 SDRAM:Burst Length (Mode Register 0).
                                                                  // # = "8", "4", "OTF".
    parameter                       CA_MIRROR               = "OFF";// C/A mirror opt for DDR3 dual rank

    // The following parameters are multiplier and divisor factors for PLLE2.
    // Based on the selected design frequency these parameters vary.
    parameter                       CLKIN_PERIOD            = 5000;// Input Clock Period
    parameter                       CLKIN_CAM_PERIOD        = CLKIN_PERIOD / 8;

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

    // Wire Declarations
    reg                                 sys_rst_n;
    wire                                sys_rst;
    reg                                 sys_clk_i;
    reg                                 cam_clk_i;
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
    wire [3:0]                          state_interface_module;
    wire                                wr_burst_data_req;

    reg                                 ISA_read_req;
	reg [DDR_ADDR_WIDTH - 1 : 0]		ISA_read_addr;
    wire [ISA_WIDTH - 1 : 0]			instruction_to_cache;

    reg                                 DATA_read_req;
    reg                                 DATA_store_req;
    reg [DATA_WIDTH - 1 : 0]            DATA_to_ddr;
	reg [DDR_ADDR_WIDTH - 1 : 0]		DATA_read_addr;
	reg [DDR_ADDR_WIDTH - 1 : 0]		DATA_write_addr;
    wire [DATA_WIDTH - 1 : 0]			DATA_to_cache;
    reg                                int;



    reg [ISA_WIDTH - 1 : 0]             MEM_ISA     [0 : TOTAL_ISA_DEPTH - 1];
    reg [DATA_WIDTH - 1 : 0]            MEM_DATA    [0 : TOTAL_DATA_DEPTH - 1];
    reg [CACHE_ISA_ADDR - 1 : 0]        MEM_ADDR    = 0;
    reg [CACHE_DATA_ADDR - 1 : 0]       MEM_ADDR_DATA = 0;
    reg [ISA_WIDTH - 1 : 0]             Instruction_reg;
    wire [ISA_WIDTH - 1 : 0]            Instruction;
    reg [DATA_WIDTH - 1 : 0]            Data_reg;
    wire [DATA_WIDTH -1 : 0]            Data;

    initial 
    begin
        $readmemb("C:/Users/42207/OneDrive/UCR/UCI/Project/AP/program/dataGenandCompiler/ISA_Bin.txt", MEM_ISA);
        $readmemb("C:/Users/42207/OneDrive/UCR/UCI/Project/AP/program/dataGenandCompiler/DATA.txt", MEM_DATA);
        /*MEM_ISA[0]      <= 0;
        MEM_ISA[1]      <= {RESET, 8'd0, 2'd0, 16'd0};
        MEM_ISA[2]      <= {LOADRBR, 8'd0, M_A, 16'h1000};
        MEM_ISA[3]      <= {LOADRBR, 8'd1, M_A, 16'h1001};
        MEM_ISA[4]      <= {LOADRBR, 8'd2, M_A, 16'h1002};
        MEM_ISA[5]      <= {LOADRBR, 8'd3, M_A, 16'h1003};
        MEM_ISA[6]      <= {LOADRBR, 8'd4, M_A, 16'h1004};
        MEM_ISA[7]      <= {LOADRBR, 8'd5, M_A, 16'h1005};
        MEM_ISA[8]      <= {LOADRBR, 8'd6, M_A, 16'h1006};
        MEM_ISA[9]      <= {LOADRBR, 8'd7, M_A, 16'h1007};
        MEM_ISA[10]     <= {LOADRBR, 8'd8, M_A, 16'h1008};
        MEM_ISA[11]     <= {LOADRBR, 8'd9, M_A, 16'h1009};
        MEM_ISA[12]     <= {LOADRBR, 8'd10, M_A, 16'h100A};
        MEM_ISA[13]     <= {LOADRBR, 8'd11, M_A, 16'h100B};
        MEM_ISA[14]     <= {LOADRBR, 8'd12, M_A, 16'h100C};
        MEM_ISA[15]     <= {LOADRBR, 8'd13, M_A, 16'h100D};
        MEM_ISA[16]     <= {LOADRBR, 8'd14, M_A, 16'h100E};
        MEM_ISA[17]     <= {LOADRBR, 8'd15, M_A, 16'h100F};

        MEM_ISA[18]     <= {LOADCBC, 8'd0, M_B, 16'h1010};
        MEM_ISA[19]     <= {LOADCBC, 8'd1, M_B, 16'h1010};
        MEM_ISA[20]     <= {LOADCBC, 8'd2, M_B, 16'h1010};
        MEM_ISA[21]     <= {LOADCBC, 8'd3, M_B, 16'h1010};
        MEM_ISA[22]     <= {LOADCBC, 8'd4, M_B, 16'h1010};
        MEM_ISA[23]     <= {LOADCBC, 8'd5, M_B, 16'h1010};
        MEM_ISA[24]     <= {LOADCBC, 8'd6, M_B, 16'h1010};
        MEM_ISA[25]     <= {LOADCBC, 8'd7, M_B, 16'h1010};

        MEM_ISA[26]     <= {ADD, 8'd0, 2'd0, 16'd0};

        MEM_ISA[27]     <= {STORERBR, 8'd0, M_B, 16'h1020};
        MEM_ISA[28]     <= {STORERBR, 8'd1, M_B, 16'h1021};
        MEM_ISA[29]     <= {STORERBR, 8'd2, M_B, 16'h1022};
        MEM_ISA[30]     <= {STORERBR, 8'd3, M_B, 16'h1023};
        MEM_ISA[31]     <= {STORERBR, 8'd4, M_B, 16'h1024};
        MEM_ISA[32]     <= {STORERBR, 8'd5, M_B, 16'h1025};
        MEM_ISA[33]     <= {STORERBR, 8'd6, M_B, 16'h1026};
        MEM_ISA[34]     <= {STORERBR, 8'd7, M_B, 16'h1027};
        MEM_ISA[35]     <= {STORERBR, 8'd8, M_B, 16'h1028};
        MEM_ISA[36]     <= {STORERBR, 8'd9, M_B, 16'h1029};
        MEM_ISA[37]     <= {STORERBR, 8'd10, M_B, 16'h102A};
        MEM_ISA[38]     <= {STORERBR, 8'd11, M_B, 16'h102B};
        MEM_ISA[39]     <= {STORERBR, 8'd12, M_B, 16'h102C};
        MEM_ISA[40]     <= {STORERBR, 8'd13, M_B, 16'h102D};
        MEM_ISA[41]     <= {STORERBR, 8'd14, M_B, 16'h102E};
        MEM_ISA[42]     <= {STORERBR, 8'd15, M_B, 16'h102F};

        MEM_ISA[43]     <= {LOADRBR, 8'd0, M_A, 16'h1030};
        MEM_ISA[44]     <= {LOADRBR, 8'd1, M_A, 16'h1031};
        MEM_ISA[45]     <= {LOADRBR, 8'd2, M_A, 16'h1032};
        MEM_ISA[46]     <= {LOADRBR, 8'd3, M_A, 16'h1033};
        MEM_ISA[47]     <= {LOADRBR, 8'd4, M_A, 16'h1034};
        MEM_ISA[48]     <= {LOADRBR, 8'd5, M_A, 16'h1035};
        MEM_ISA[49]     <= {LOADRBR, 8'd6, M_A, 16'h1036};
        MEM_ISA[50]     <= {LOADRBR, 8'd7, M_A, 16'h1037};
        MEM_ISA[51]     <= {LOADRBR, 8'd8, M_A, 16'h1038};
        MEM_ISA[52]     <= {LOADRBR, 8'd9, M_A, 16'h1039};
        MEM_ISA[53]     <= {LOADRBR, 8'd10, M_A, 16'h103A};
        MEM_ISA[54]     <= {LOADRBR, 8'd11, M_A, 16'h103B};
        MEM_ISA[55]     <= {LOADRBR, 8'd12, M_A, 16'h103C};
        MEM_ISA[56]     <= {LOADRBR, 8'd13, M_A, 16'h103D};
        MEM_ISA[57]     <= {LOADRBR, 8'd14, M_A, 16'h103E};
        MEM_ISA[58]     <= {LOADRBR, 8'd15, M_A, 16'h103F};

        MEM_ISA[59]     <= {ADD, 8'd0, 2'd0, 16'd0};
        MEM_ISA[60]     <= {COPY, 6'd0, M_A, M_B, 16'h0000};
        MEM_ISA[61]     <= {ADD, 8'd0, 2'd0, 16'd0};
        MEM_ISA[62]     <= {COPY, 6'd0, M_A, M_B, 16'h0000};
        MEM_ISA[63]     <= {ADD, 8'd0, 2'd0, 16'd0};

        MEM_ISA[64]     <= {STORECBC, 8'd0, M_B, 16'h1040};
        MEM_ISA[65]     <= {STORECBC, 8'd1, M_B, 16'h1040};
        MEM_ISA[66]     <= {STORECBC, 8'd2, M_B, 16'h1040};
        MEM_ISA[67]     <= {STORECBC, 8'd3, M_B, 16'h1040};
        MEM_ISA[68]     <= {STORECBC, 8'd4, M_B, 16'h1040};
        MEM_ISA[69]     <= {STORECBC, 8'd5, M_B, 16'h1040};
        MEM_ISA[70]     <= {STORECBC, 8'd6, M_B, 16'h1040};
        MEM_ISA[71]     <= {STORECBC, 8'd7, M_B, 16'h1040};

        MEM_ISA[72]     <= {LOADCBC, 8'd0, M_A, 16'h1020};
        MEM_ISA[73]     <= {LOADCBC, 8'd1, M_A, 16'h1020};
        MEM_ISA[74]     <= {LOADCBC, 8'd2, M_A, 16'h1020};
        MEM_ISA[75]     <= {LOADCBC, 8'd3, M_A, 16'h1020};
        MEM_ISA[76]     <= {LOADCBC, 8'd4, M_A, 16'h1020};
        MEM_ISA[77]     <= {LOADCBC, 8'd5, M_A, 16'h1020};
        MEM_ISA[78]     <= {LOADCBC, 8'd6, M_A, 16'h1020};
        MEM_ISA[79]     <= {LOADCBC, 8'd7, M_A, 16'h1020};

        MEM_ISA[80]     <= {SUB, 8'd0, 2'd0, 16'd0}; 

        MEM_ISA[81]     <= {STORECBC, 8'd0, M_B, 16'h1050};
        MEM_ISA[82]     <= {STORECBC, 8'd1, M_B, 16'h1050};
        MEM_ISA[83]     <= {STORECBC, 8'd2, M_B, 16'h1050};
        MEM_ISA[84]     <= {STORECBC, 8'd3, M_B, 16'h1050};
        MEM_ISA[85]     <= {STORECBC, 8'd4, M_B, 16'h1050};
        MEM_ISA[86]     <= {STORECBC, 8'd5, M_B, 16'h1050};
        MEM_ISA[87]     <= {STORECBC, 8'd6, M_B, 16'h1050};
        MEM_ISA[88]     <= {STORECBC, 8'd7, M_B, 16'h1050};

        MEM_ISA[89]     <= {SUB, 8'd0, 2'd0, 16'd0}; 

        MEM_ISA[90]     <= {STORECBC, 8'd0, M_B, 16'h1060};
        MEM_ISA[91]     <= {STORECBC, 8'd1, M_B, 16'h1060};
        MEM_ISA[92]     <= {STORECBC, 8'd2, M_B, 16'h1060};
        MEM_ISA[93]     <= {STORECBC, 8'd3, M_B, 16'h1060};
        MEM_ISA[94]     <= {STORECBC, 8'd4, M_B, 16'h1060};
        MEM_ISA[95]     <= {STORECBC, 8'd5, M_B, 16'h1060};
        MEM_ISA[96]     <= {STORECBC, 8'd6, M_B, 16'h1060};
        MEM_ISA[97]     <= {STORECBC, 8'd7, M_B, 16'h1060};

        MEM_ISA[98]     <= {LOADRBR, 8'd0, M_A, 16'h1040};
        MEM_ISA[99]     <= {LOADRBR, 8'd1, M_A, 16'h1041};
        MEM_ISA[100]     <= {LOADRBR, 8'd2, M_A, 16'h1042};
        MEM_ISA[101]     <= {LOADRBR, 8'd3, M_A, 16'h1043};
        MEM_ISA[102]     <= {LOADRBR, 8'd4, M_A, 16'h1044};
        MEM_ISA[103]     <= {LOADRBR, 8'd5, M_A, 16'h1045};
        MEM_ISA[104]     <= {LOADRBR, 8'd6, M_A, 16'h1046};
        MEM_ISA[105]     <= {LOADRBR, 8'd7, M_A, 16'h1047};
        MEM_ISA[106]     <= {LOADRBR, 8'd8, M_A, 16'h1048};
        MEM_ISA[107]     <= {LOADRBR, 8'd9, M_A, 16'h1049};
        MEM_ISA[108]     <= {LOADRBR, 8'd10, M_A, 16'h104A};
        MEM_ISA[109]     <= {LOADRBR, 8'd11, M_A, 16'h104B};
        MEM_ISA[110]     <= {LOADRBR, 8'd12, M_A, 16'h104C};
        MEM_ISA[111]     <= {LOADRBR, 8'd13, M_A, 16'h104D};
        MEM_ISA[112]     <= {LOADRBR, 8'd14, M_A, 16'h104E};
        MEM_ISA[113]     <= {LOADRBR, 8'd15, M_A, 16'h104F};

        MEM_ISA[114]     <= {ABS, 8'd0, 2'd0, 16'd0}; 

        MEM_ISA[115]     <= {STORECBC, 8'd0, M_R, 16'h1080};
        MEM_ISA[116]     <= {STORECBC, 8'd1, M_R, 16'h1080};
        MEM_ISA[117]     <= {STORECBC, 8'd2, M_R, 16'h1080};
        MEM_ISA[118]     <= {STORECBC, 8'd3, M_R, 16'h1080};
        MEM_ISA[119]     <= {STORECBC, 8'd4, M_R, 16'h1080};
        MEM_ISA[120]     <= {STORECBC, 8'd5, M_R, 16'h1080};
        MEM_ISA[121]     <= {STORECBC, 8'd6, M_R, 16'h1080};
        MEM_ISA[122]     <= {STORECBC, 8'd7, M_R, 16'h1080};

        MEM_ISA[123]     <= {TSC, 8'd0, 2'd0, 16'd0}; 

        MEM_ISA[124]     <= {STORERBR, 8'd0, M_R, 16'h102C};
        MEM_ISA[125]     <= {STORERBR, 8'd1, M_R, 16'h102D};
        MEM_ISA[126]     <= {STORERBR, 8'd2, M_R, 16'h102E};
        MEM_ISA[127]     <= {STORERBR, 8'd3, M_R, 16'h102F};*/
        
    end

    always @(posedge sys_clk_i) 
    begin
        if(wr_burst_data_req & (state_interface_module == MEM_WRITE_ISA))
        begin
          Instruction_reg = MEM_ISA[MEM_ADDR];
          MEM_ADDR = MEM_ADDR + 1;
        end

        else if(wr_burst_data_req & (state_interface_module == MEM_WRITE_DATA))
        begin
          Data_reg = MEM_DATA[MEM_ADDR_DATA];
          MEM_ADDR_DATA = MEM_ADDR_DATA + 1;
        end
    end

    assign Instruction = Instruction_reg;
    assign Data = Data_reg;

    initial begin
        int = 0;
        #57858500
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
        sys_clk_i = 1'b0;
    always
        sys_clk_i = #(CLKIN_PERIOD/2.0) ~sys_clk_i;

    initial 
        cam_clk_i = 1'b1;
    always
        cam_clk_i = #(CLKIN_CAM_PERIOD/2.0) ~cam_clk_i;

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
        .sys_clk_i              (sys_clk_i),
        .cam_clk_i              (cam_clk_i),
        .int                    (int),
        .init_calib_complete    (init_calib_complete),
        .sys_rst                (sys_rst),
        .Instruction            (Instruction),
        .Data                   (Data),
        .wr_burst_data_req      (wr_burst_data_req),
        .state_interface_module (state_interface_module),
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



