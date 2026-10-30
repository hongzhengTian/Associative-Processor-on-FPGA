// Top level of the AP SoC for a Zynq-7000 block design (module reference).
//
// Plain Verilog so that Vivado can use it directly as a block design module.
// The port names follow the AXI conventions, so Vivado infers the interfaces:
//
//   s_axi_ctrl  AXI4-Lite slave, 128 KiB: control registers and program RAM
//               (see ap_host_bridge.sv), connect to M_AXI_GP0 of the PS
//   m_axi_mem   AXI4 master, 32-bit: the SoC's external memory window,
//               connect to S_AXI_HP0 of the PS (DDR)
//
// Everything runs on aclk; aresetn is the usual active-low peripheral reset.
module ap_zynq #(
    parameter ROWS = 512,
    parameter COLS = 64
) (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi_ctrl:m_axi_mem, ASSOCIATED_RESET aresetn" *)
    input  wire        aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        aresetn,

    input  wire [16:0] s_axi_ctrl_awaddr,
    input  wire        s_axi_ctrl_awvalid,
    output wire        s_axi_ctrl_awready,
    input  wire [31:0] s_axi_ctrl_wdata,
    input  wire [3:0]  s_axi_ctrl_wstrb,
    input  wire        s_axi_ctrl_wvalid,
    output wire        s_axi_ctrl_wready,
    output wire [1:0]  s_axi_ctrl_bresp,
    output wire        s_axi_ctrl_bvalid,
    input  wire        s_axi_ctrl_bready,
    input  wire [16:0] s_axi_ctrl_araddr,
    input  wire        s_axi_ctrl_arvalid,
    output wire        s_axi_ctrl_arready,
    output wire [31:0] s_axi_ctrl_rdata,
    output wire [1:0]  s_axi_ctrl_rresp,
    output wire        s_axi_ctrl_rvalid,
    input  wire        s_axi_ctrl_rready,

    output wire [31:0] m_axi_mem_awaddr,
    output wire [7:0]  m_axi_mem_awlen,
    output wire [2:0]  m_axi_mem_awsize,
    output wire [1:0]  m_axi_mem_awburst,
    output wire [3:0]  m_axi_mem_awcache,
    output wire [2:0]  m_axi_mem_awprot,
    output wire        m_axi_mem_awvalid,
    input  wire        m_axi_mem_awready,
    output wire [31:0] m_axi_mem_wdata,
    output wire [3:0]  m_axi_mem_wstrb,
    output wire        m_axi_mem_wlast,
    output wire        m_axi_mem_wvalid,
    input  wire        m_axi_mem_wready,
    input  wire [1:0]  m_axi_mem_bresp,
    input  wire        m_axi_mem_bvalid,
    output wire        m_axi_mem_bready,
    output wire [31:0] m_axi_mem_araddr,
    output wire [7:0]  m_axi_mem_arlen,
    output wire [2:0]  m_axi_mem_arsize,
    output wire [1:0]  m_axi_mem_arburst,
    output wire [3:0]  m_axi_mem_arcache,
    output wire [2:0]  m_axi_mem_arprot,
    output wire        m_axi_mem_arvalid,
    input  wire        m_axi_mem_arready,
    input  wire [31:0] m_axi_mem_rdata,
    input  wire [1:0]  m_axi_mem_rresp,
    input  wire        m_axi_mem_rlast,
    input  wire        m_axi_mem_rvalid,
    output wire        m_axi_mem_rready
);

    // normal, non-cacheable bufferable memory; unprivileged data access
    assign m_axi_mem_awcache = 4'b0011;
    assign m_axi_mem_arcache = 4'b0011;
    assign m_axi_mem_awprot  = 3'b000;
    assign m_axi_mem_arprot  = 3'b000;

    wire        soc_rst_n, host_sel, host_valid, host_ready, trap;
    wire [31:0] host_addr, host_wdata, host_rdata;
    wire [3:0]  host_wstrb;
    wire        io_valid, io_ready;
    wire [31:0] io_addr, io_wdata, io_rdata;
    wire [3:0]  io_wstrb;

    wire [31:0] e_awaddr, e_wdata, e_araddr, e_rdata;
    wire [7:0]  e_awlen, e_arlen;
    wire [2:0]  e_awsize, e_arsize;
    wire [1:0]  e_awburst, e_arburst, e_bresp, e_rresp;
    wire [3:0]  e_wstrb;
    wire        e_awvalid, e_awready, e_wlast, e_wvalid, e_wready;
    wire        e_bvalid, e_bready, e_arvalid, e_arready;
    wire        e_rlast, e_rvalid, e_rready;

    ap_soc #(.ROWS(ROWS), .COLS(COLS)) u_soc (
        .clk           (aclk),
        .rst_n         (soc_rst_n),
        .trap          (trap),
        .host_sel      (host_sel),
        .host_valid    (host_valid),
        .host_ready    (host_ready),
        .host_addr     (host_addr),
        .host_wdata    (host_wdata),
        .host_wstrb    (host_wstrb),
        .host_rdata    (host_rdata),
        .io_valid      (io_valid),
        .io_addr       (io_addr),
        .io_wdata      (io_wdata),
        .io_wstrb      (io_wstrb),
        .io_ready      (io_ready),
        .io_rdata      (io_rdata),
        .m_axi_awaddr  (e_awaddr),
        .m_axi_awlen   (e_awlen),
        .m_axi_awsize  (e_awsize),
        .m_axi_awburst (e_awburst),
        .m_axi_awvalid (e_awvalid),
        .m_axi_awready (e_awready),
        .m_axi_wdata   (e_wdata),
        .m_axi_wstrb   (e_wstrb),
        .m_axi_wlast   (e_wlast),
        .m_axi_wvalid  (e_wvalid),
        .m_axi_wready  (e_wready),
        .m_axi_bresp   (e_bresp),
        .m_axi_bvalid  (e_bvalid),
        .m_axi_bready  (e_bready),
        .m_axi_araddr  (e_araddr),
        .m_axi_arlen   (e_arlen),
        .m_axi_arsize  (e_arsize),
        .m_axi_arburst (e_arburst),
        .m_axi_arvalid (e_arvalid),
        .m_axi_arready (e_arready),
        .m_axi_rdata   (e_rdata),
        .m_axi_rresp   (e_rresp),
        .m_axi_rlast   (e_rlast),
        .m_axi_rvalid  (e_rvalid),
        .m_axi_rready  (e_rready)
    );

    ap_host_bridge #(.ROWS(ROWS), .COLS(COLS)) u_bridge (
        .clk        (aclk),
        .rst_n      (aresetn),
        .s_awaddr   (s_axi_ctrl_awaddr),
        .s_awvalid  (s_axi_ctrl_awvalid),
        .s_awready  (s_axi_ctrl_awready),
        .s_wdata    (s_axi_ctrl_wdata),
        .s_wstrb    (s_axi_ctrl_wstrb),
        .s_wvalid   (s_axi_ctrl_wvalid),
        .s_wready   (s_axi_ctrl_wready),
        .s_bresp    (s_axi_ctrl_bresp),
        .s_bvalid   (s_axi_ctrl_bvalid),
        .s_bready   (s_axi_ctrl_bready),
        .s_araddr   (s_axi_ctrl_araddr),
        .s_arvalid  (s_axi_ctrl_arvalid),
        .s_arready  (s_axi_ctrl_arready),
        .s_rdata    (s_axi_ctrl_rdata),
        .s_rresp    (s_axi_ctrl_rresp),
        .s_rvalid   (s_axi_ctrl_rvalid),
        .s_rready   (s_axi_ctrl_rready),
        .soc_rst_n  (soc_rst_n),
        .host_sel   (host_sel),
        .host_valid (host_valid),
        .host_ready (host_ready),
        .host_addr  (host_addr),
        .host_wdata (host_wdata),
        .host_wstrb (host_wstrb),
        .host_rdata (host_rdata),
        .trap       (trap),
        .io_valid   (io_valid),
        .io_addr    (io_addr),
        .io_wdata   (io_wdata),
        .io_wstrb   (io_wstrb),
        .io_ready   (io_ready),
        .io_rdata   (io_rdata),
        .e_awaddr   (e_awaddr),
        .e_awlen    (e_awlen),
        .e_awsize   (e_awsize),
        .e_awburst  (e_awburst),
        .e_awvalid  (e_awvalid),
        .e_awready  (e_awready),
        .e_wdata    (e_wdata),
        .e_wstrb    (e_wstrb),
        .e_wlast    (e_wlast),
        .e_wvalid   (e_wvalid),
        .e_wready   (e_wready),
        .e_bresp    (e_bresp),
        .e_bvalid   (e_bvalid),
        .e_bready   (e_bready),
        .e_araddr   (e_araddr),
        .e_arlen    (e_arlen),
        .e_arsize   (e_arsize),
        .e_arburst  (e_arburst),
        .e_arvalid  (e_arvalid),
        .e_arready  (e_arready),
        .e_rdata    (e_rdata),
        .e_rresp    (e_rresp),
        .e_rlast    (e_rlast),
        .e_rvalid   (e_rvalid),
        .e_rready   (e_rready),
        .m_awaddr   (m_axi_mem_awaddr),
        .m_awlen    (m_axi_mem_awlen),
        .m_awsize   (m_axi_mem_awsize),
        .m_awburst  (m_axi_mem_awburst),
        .m_awvalid  (m_axi_mem_awvalid),
        .m_awready  (m_axi_mem_awready),
        .m_wdata    (m_axi_mem_wdata),
        .m_wstrb    (m_axi_mem_wstrb),
        .m_wlast    (m_axi_mem_wlast),
        .m_wvalid   (m_axi_mem_wvalid),
        .m_wready   (m_axi_mem_wready),
        .m_bresp    (m_axi_mem_bresp),
        .m_bvalid   (m_axi_mem_bvalid),
        .m_bready   (m_axi_mem_bready),
        .m_araddr   (m_axi_mem_araddr),
        .m_arlen    (m_axi_mem_arlen),
        .m_arsize   (m_axi_mem_arsize),
        .m_arburst  (m_axi_mem_arburst),
        .m_arvalid  (m_axi_mem_arvalid),
        .m_arready  (m_axi_mem_arready),
        .m_rdata    (m_axi_mem_rdata),
        .m_rresp    (m_axi_mem_rresp),
        .m_rlast    (m_axi_mem_rlast),
        .m_rvalid   (m_axi_mem_rvalid),
        .m_rready   (m_axi_mem_rready)
    );

endmodule
