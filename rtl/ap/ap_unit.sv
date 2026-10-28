// AP unit: the AP core plus its DMA engine, behind one command interface.
//
// DMA commands (GRP_DMA) go to ap_dma, everything else to ap_core. While a
// transfer runs, ap_dma drives the core's command port itself (XPUT/XGET)
// and the unit accepts no new command. Interface timing as in ap_core.
module ap_unit #(
    parameter int ROWS   = 256,
    parameter int COLS   = 128,
    parameter int ADDR_W = 32
) (
    input  logic              clk,
    input  logic              rst_n,

    input  logic              cmd_valid,
    output logic              cmd_ready,
    input  logic [2:0]        cmd_funct3,
    input  logic [6:0]        cmd_funct7,
    input  logic [31:0]       cmd_rs1,
    input  logic [31:0]       cmd_rs2,
    output logic              cmd_legal,

    output logic              rsp_valid,
    output logic              rsp_wr,
    output logic [31:0]       rsp_data,

    // AXI4 master of the DMA engine
    output logic [ADDR_W-1:0] m_axi_awaddr,
    output logic [7:0]        m_axi_awlen,
    output logic [2:0]        m_axi_awsize,
    output logic [1:0]        m_axi_awburst,
    output logic              m_axi_awvalid,
    input  logic              m_axi_awready,
    output logic [31:0]       m_axi_wdata,
    output logic [3:0]        m_axi_wstrb,
    output logic              m_axi_wlast,
    output logic              m_axi_wvalid,
    input  logic              m_axi_wready,
    input  logic [1:0]        m_axi_bresp,
    input  logic              m_axi_bvalid,
    output logic              m_axi_bready,
    output logic [ADDR_W-1:0] m_axi_araddr,
    output logic [7:0]        m_axi_arlen,
    output logic [2:0]        m_axi_arsize,
    output logic [1:0]        m_axi_arburst,
    output logic              m_axi_arvalid,
    input  logic              m_axi_arready,
    input  logic [31:0]       m_axi_rdata,
    input  logic [1:0]        m_axi_rresp,
    input  logic              m_axi_rlast,
    input  logic              m_axi_rvalid,
    output logic              m_axi_rready,

    output logic              dma_busy
);

    logic        is_dma;
    logic        core_legal;

    // core command port, shared by the unit input and the DMA engine
    logic        core_cmd_valid, core_cmd_ready;
    logic [2:0]  core_cmd_funct3;
    logic [6:0]  core_cmd_funct7;
    logic [31:0] core_cmd_rs1, core_cmd_rs2;
    logic        core_rsp_valid, core_rsp_wr;
    logic [31:0] core_rsp_data;

    logic        dma_cmd_ready;
    logic        dma_core_valid;
    logic [2:0]  dma_core_funct3;
    logic [6:0]  dma_core_funct7;
    logic [31:0] dma_core_rs1, dma_core_rs2;
    logic        dma_rsp_valid, dma_rsp_wr;
    logic [31:0] dma_rsp_data;

    logic        xbuf_we;
    logic [4:0]  xbuf_idx;
    logic [31:0] xbuf_wdata, xbuf_rdata;

    assign is_dma    = (cmd_funct3 == ap_pkg::GRP_DMA);
    assign cmd_legal = is_dma ? (cmd_funct7 <= ap_pkg::DMA_STORE) : core_legal;
    assign cmd_ready = core_cmd_ready && dma_cmd_ready;

    assign core_cmd_valid  = dma_busy ? dma_core_valid  : (cmd_valid && !is_dma);
    assign core_cmd_funct3 = dma_busy ? dma_core_funct3 : cmd_funct3;
    assign core_cmd_funct7 = dma_busy ? dma_core_funct7 : cmd_funct7;
    assign core_cmd_rs1    = dma_busy ? dma_core_rs1    : cmd_rs1;
    assign core_cmd_rs2    = dma_busy ? dma_core_rs2    : cmd_rs2;

    // Core responses during a transfer belong to the DMA engine.
    assign rsp_valid = (core_rsp_valid && !dma_busy) || dma_rsp_valid;
    assign rsp_wr    = dma_rsp_valid ? dma_rsp_wr   : core_rsp_wr;
    assign rsp_data  = dma_rsp_valid ? dma_rsp_data : core_rsp_data;

    ap_core #(.ROWS(ROWS), .COLS(COLS)) u_core (
        .clk        (clk),
        .rst_n      (rst_n),
        .cmd_valid  (core_cmd_valid),
        .cmd_ready  (core_cmd_ready),
        .cmd_funct3 (core_cmd_funct3),
        .cmd_funct7 (core_cmd_funct7),
        .cmd_rs1    (core_cmd_rs1),
        .cmd_rs2    (core_cmd_rs2),
        .cmd_legal  (core_legal),
        .rsp_valid  (core_rsp_valid),
        .rsp_wr     (core_rsp_wr),
        .rsp_data   (core_rsp_data),
        .xbuf_we    (xbuf_we),
        .xbuf_idx   (xbuf_idx),
        .xbuf_wdata (xbuf_wdata),
        .xbuf_rdata (xbuf_rdata)
    );

    ap_dma #(.ADDR_W(ADDR_W)) u_dma (
        .clk             (clk),
        .rst_n           (rst_n),
        .cmd_valid       (cmd_valid && is_dma && core_cmd_ready),
        .cmd_ready       (dma_cmd_ready),
        .cmd_funct7      (cmd_funct7),
        .cmd_rs1         (cmd_rs1),
        .cmd_rs2         (cmd_rs2),
        .rsp_valid       (dma_rsp_valid),
        .rsp_wr          (dma_rsp_wr),
        .rsp_data        (dma_rsp_data),
        .busy            (dma_busy),
        .core_cmd_valid  (dma_core_valid),
        .core_cmd_ready  (core_cmd_ready),
        .core_cmd_funct3 (dma_core_funct3),
        .core_cmd_funct7 (dma_core_funct7),
        .core_cmd_rs1    (dma_core_rs1),
        .core_cmd_rs2    (dma_core_rs2),
        .core_rsp_valid  (core_rsp_valid),
        .xbuf_we         (xbuf_we),
        .xbuf_idx        (xbuf_idx),
        .xbuf_wdata      (xbuf_wdata),
        .xbuf_rdata      (xbuf_rdata),
        .m_axi_awaddr    (m_axi_awaddr),
        .m_axi_awlen     (m_axi_awlen),
        .m_axi_awsize    (m_axi_awsize),
        .m_axi_awburst   (m_axi_awburst),
        .m_axi_awvalid   (m_axi_awvalid),
        .m_axi_awready   (m_axi_awready),
        .m_axi_wdata     (m_axi_wdata),
        .m_axi_wstrb     (m_axi_wstrb),
        .m_axi_wlast     (m_axi_wlast),
        .m_axi_wvalid    (m_axi_wvalid),
        .m_axi_wready    (m_axi_wready),
        .m_axi_bresp     (m_axi_bresp),
        .m_axi_bvalid    (m_axi_bvalid),
        .m_axi_bready    (m_axi_bready),
        .m_axi_araddr    (m_axi_araddr),
        .m_axi_arlen     (m_axi_arlen),
        .m_axi_arsize    (m_axi_arsize),
        .m_axi_arburst   (m_axi_arburst),
        .m_axi_arvalid   (m_axi_arvalid),
        .m_axi_arready   (m_axi_arready),
        .m_axi_rdata     (m_axi_rdata),
        .m_axi_rresp     (m_axi_rresp),
        .m_axi_rlast     (m_axi_rlast),
        .m_axi_rvalid    (m_axi_rvalid),
        .m_axi_rready    (m_axi_rready)
    );

endmodule
