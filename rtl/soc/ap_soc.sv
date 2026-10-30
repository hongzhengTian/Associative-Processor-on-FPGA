// AP system on chip: PicoRV32 (RV32IM) + AP unit + on-chip RAM.
//
// Address map (CPU view):
//   0x0000_0000  on-chip RAM, RAM_WORDS x 32 bit, program and data
//   0x1000_0000  I/O port (io_*), decoded outside: console, exit, ...
//   0x8000_0000  external memory window, mapped to m_axi at address & 0x7FFF_FFFF
//
// The CPU issues AP instructions (custom-1) through PCPI and stalls until
// each finishes. The AP DMA engine and the CPU's own external accesses share
// m_axi; they never overlap because the CPU waits for DMA instructions.
//
// The host port (host_*) lets a host processor load the program RAM. It owns
// the RAM while host_sel is high, which is only allowed while rst_n holds the
// SoC in reset. Tie host_sel low when the RAM is loaded otherwise.
module ap_soc #(
    parameter int    ROWS      = 256,
    parameter int    COLS      = 128,
    parameter int    RAM_WORDS = 16384,     // 64 KiB
    parameter        INIT_FILE = ""
) (
    input  logic        clk,
    input  logic        rst_n,
    output logic        trap,

    // host access to the program RAM (same handshake as the CPU bus)
    input  logic        host_sel,
    input  logic        host_valid,
    output logic        host_ready,
    input  logic [31:0] host_addr,
    input  logic [31:0] host_wdata,
    input  logic [3:0]  host_wstrb,
    output logic [31:0] host_rdata,

    // I/O port, one access at a time, valid held until ready
    output logic        io_valid,
    output logic [31:0] io_addr,
    output logic [31:0] io_wdata,
    output logic [3:0]  io_wstrb,
    input  logic        io_ready,
    input  logic [31:0] io_rdata,

    // external memory, AXI4, 32-bit data
    output logic [31:0] m_axi_awaddr,
    output logic [7:0]  m_axi_awlen,
    output logic [2:0]  m_axi_awsize,
    output logic [1:0]  m_axi_awburst,
    output logic        m_axi_awvalid,
    input  logic        m_axi_awready,
    output logic [31:0] m_axi_wdata,
    output logic [3:0]  m_axi_wstrb,
    output logic        m_axi_wlast,
    output logic        m_axi_wvalid,
    input  logic        m_axi_wready,
    input  logic [1:0]  m_axi_bresp,
    input  logic        m_axi_bvalid,
    output logic        m_axi_bready,
    output logic [31:0] m_axi_araddr,
    output logic [7:0]  m_axi_arlen,
    output logic [2:0]  m_axi_arsize,
    output logic [1:0]  m_axi_arburst,
    output logic        m_axi_arvalid,
    input  logic        m_axi_arready,
    input  logic [31:0] m_axi_rdata,
    input  logic [1:0]  m_axi_rresp,
    input  logic        m_axi_rlast,
    input  logic        m_axi_rvalid,
    output logic        m_axi_rready
);

    localparam logic [31:0] RAM_BYTES = 32'(RAM_WORDS * 4);

    // ------------------------------------------------------------------ CPU
    logic        mem_valid, mem_instr, mem_ready;
    logic [31:0] mem_addr, mem_wdata, mem_rdata;
    logic [3:0]  mem_wstrb;

    logic        pcpi_valid, pcpi_wr, pcpi_wait, pcpi_ready;
    logic [31:0] pcpi_insn, pcpi_rs1, pcpi_rs2, pcpi_rd;

    /* verilator lint_off PINCONNECTEMPTY */
    picorv32 #(
        .ENABLE_COUNTERS (1),
        .ENABLE_REGS_DUALPORT (1),
        .BARREL_SHIFTER  (1),
        .COMPRESSED_ISA  (0),
        .CATCH_MISALIGN  (1),
        .CATCH_ILLINSN   (1),
        .ENABLE_PCPI     (1),
        .ENABLE_MUL      (1),
        .ENABLE_DIV      (1),
        .ENABLE_IRQ      (0),
        .PROGADDR_RESET  (32'h0000_0000),
        .STACKADDR       (RAM_BYTES)
    ) u_cpu (
        .clk          (clk),
        .resetn       (rst_n),
        .trap         (trap),
        .mem_valid    (mem_valid),
        .mem_instr    (mem_instr),
        .mem_ready    (mem_ready),
        .mem_addr     (mem_addr),
        .mem_wdata    (mem_wdata),
        .mem_wstrb    (mem_wstrb),
        .mem_rdata    (mem_rdata),
        .mem_la_read  (),
        .mem_la_write (),
        .mem_la_addr  (),
        .mem_la_wdata (),
        .mem_la_wstrb (),
        .pcpi_valid   (pcpi_valid),
        .pcpi_insn    (pcpi_insn),
        .pcpi_rs1     (pcpi_rs1),
        .pcpi_rs2     (pcpi_rs2),
        .pcpi_wr      (pcpi_wr),
        .pcpi_rd      (pcpi_rd),
        .pcpi_wait    (pcpi_wait),
        .pcpi_ready   (pcpi_ready),
        .irq          (32'd0),
        .eoi          (),
        .trace_valid  (),
        .trace_data   ()
    );
    /* verilator lint_on PINCONNECTEMPTY */

    // ---------------------------------------------------------- bus decode
    logic sel_ram, sel_io, sel_ext, sel_none;
    assign sel_ram  = (mem_addr < RAM_BYTES);
    assign sel_io   = (mem_addr[31:28] == 4'h1);
    assign sel_ext  = mem_addr[31];
    assign sel_none = !(sel_ram || sel_io || sel_ext);

    logic        ram_ready, ext_ready, none_ready;
    logic [31:0] ram_rdata, ext_rdata;

    logic ram_port_ready;

    soc_ram #(.WORDS(RAM_WORDS), .INIT_FILE(INIT_FILE)) u_ram (
        .clk   (clk),
        .valid (host_sel ? host_valid : (mem_valid && sel_ram)),
        .ready (ram_port_ready),
        .addr  (host_sel ? host_addr  : mem_addr),
        .wdata (host_sel ? host_wdata : mem_wdata),
        .wstrb (host_sel ? host_wstrb : mem_wstrb),
        .rdata (ram_rdata)
    );

    assign ram_ready  = ram_port_ready && !host_sel;
    assign host_ready = ram_port_ready && host_sel;
    assign host_rdata = ram_rdata;

    assign io_valid = mem_valid && sel_io;
    assign io_addr  = mem_addr;
    assign io_wdata = mem_wdata;
    assign io_wstrb = mem_wstrb;

    // Unmapped addresses answer at once and read as zero.
    always_ff @(posedge clk) none_ready <= mem_valid && sel_none && !none_ready;

    assign mem_ready = ram_ready || (io_valid && io_ready) || ext_ready || none_ready;
    assign mem_rdata = sel_ram ? ram_rdata :
                       sel_io  ? io_rdata  :
                       sel_ext ? ext_rdata : 32'd0;

    // ------------------------------------------------------------ AP unit
    logic        ap_cmd_valid, ap_cmd_ready, ap_cmd_legal;
    logic [2:0]  ap_cmd_funct3;
    logic [6:0]  ap_cmd_funct7;
    logic [31:0] ap_cmd_rs1, ap_cmd_rs2;
    logic        ap_rsp_valid, ap_rsp_wr;
    logic [31:0] ap_rsp_data;
    logic        dma_busy;

    ap_pcpi u_pcpi (
        .clk        (clk),
        .rst_n      (rst_n),
        .pcpi_valid (pcpi_valid),
        .pcpi_insn  (pcpi_insn),
        .pcpi_rs1   (pcpi_rs1),
        .pcpi_rs2   (pcpi_rs2),
        .pcpi_wr    (pcpi_wr),
        .pcpi_rd    (pcpi_rd),
        .pcpi_wait  (pcpi_wait),
        .pcpi_ready (pcpi_ready),
        .cmd_valid  (ap_cmd_valid),
        .cmd_ready  (ap_cmd_ready),
        .cmd_funct3 (ap_cmd_funct3),
        .cmd_funct7 (ap_cmd_funct7),
        .cmd_rs1    (ap_cmd_rs1),
        .cmd_rs2    (ap_cmd_rs2),
        .cmd_legal  (ap_cmd_legal),
        .rsp_valid  (ap_rsp_valid),
        .rsp_wr     (ap_rsp_wr),
        .rsp_data   (ap_rsp_data)
    );

    // DMA side of the external memory port
    logic [31:0] d_awaddr, d_araddr, d_wdata, d_rdata;
    logic [7:0]  d_awlen, d_arlen;
    logic [2:0]  d_awsize, d_arsize;
    logic [1:0]  d_awburst, d_arburst, d_bresp, d_rresp;
    logic [3:0]  d_wstrb;
    logic        d_awvalid, d_awready, d_wlast, d_wvalid, d_wready;
    logic        d_bvalid, d_bready, d_arvalid, d_arready;
    logic        d_rlast, d_rvalid, d_rready;

    ap_unit #(.ROWS(ROWS), .COLS(COLS), .ADDR_W(32)) u_ap (
        .clk           (clk),
        .rst_n         (rst_n),
        .cmd_valid     (ap_cmd_valid),
        .cmd_ready     (ap_cmd_ready),
        .cmd_funct3    (ap_cmd_funct3),
        .cmd_funct7    (ap_cmd_funct7),
        .cmd_rs1       (ap_cmd_rs1),
        .cmd_rs2       (ap_cmd_rs2),
        .cmd_legal     (ap_cmd_legal),
        .rsp_valid     (ap_rsp_valid),
        .rsp_wr        (ap_rsp_wr),
        .rsp_data      (ap_rsp_data),
        .m_axi_awaddr  (d_awaddr),
        .m_axi_awlen   (d_awlen),
        .m_axi_awsize  (d_awsize),
        .m_axi_awburst (d_awburst),
        .m_axi_awvalid (d_awvalid),
        .m_axi_awready (d_awready),
        .m_axi_wdata   (d_wdata),
        .m_axi_wstrb   (d_wstrb),
        .m_axi_wlast   (d_wlast),
        .m_axi_wvalid  (d_wvalid),
        .m_axi_wready  (d_wready),
        .m_axi_bresp   (d_bresp),
        .m_axi_bvalid  (d_bvalid),
        .m_axi_bready  (d_bready),
        .m_axi_araddr  (d_araddr),
        .m_axi_arlen   (d_arlen),
        .m_axi_arsize  (d_arsize),
        .m_axi_arburst (d_arburst),
        .m_axi_arvalid (d_arvalid),
        .m_axi_arready (d_arready),
        .m_axi_rdata   (d_rdata),
        .m_axi_rresp   (d_rresp),
        .m_axi_rlast   (d_rlast),
        .m_axi_rvalid  (d_rvalid),
        .m_axi_rready  (d_rready),
        .dma_busy      (dma_busy)
    );

    // CPU side of the external memory port (single beats)
    logic        c_awvalid, c_awready, c_wvalid, c_wready, c_bvalid, c_bready;
    logic        c_arvalid, c_arready, c_rvalid, c_rready;
    logic [31:0] c_awaddr, c_araddr, c_wdata;
    logic [3:0]  c_wstrb;

    /* verilator lint_off PINCONNECTEMPTY */
    picorv32_axi_adapter u_cpu_axi (
        .clk             (clk),
        .resetn          (rst_n),
        .mem_axi_awvalid (c_awvalid),
        .mem_axi_awready (c_awready),
        .mem_axi_awaddr  (c_awaddr),
        .mem_axi_awprot  (),
        .mem_axi_wvalid  (c_wvalid),
        .mem_axi_wready  (c_wready),
        .mem_axi_wdata   (c_wdata),
        .mem_axi_wstrb   (c_wstrb),
        .mem_axi_bvalid  (c_bvalid),
        .mem_axi_bready  (c_bready),
        .mem_axi_arvalid (c_arvalid),
        .mem_axi_arready (c_arready),
        .mem_axi_araddr  (c_araddr),
        .mem_axi_arprot  (),
        .mem_axi_rvalid  (c_rvalid),
        .mem_axi_rready  (c_rready),
        .mem_axi_rdata   (m_axi_rdata),
        .mem_valid       (mem_valid && sel_ext),
        .mem_instr       (mem_instr),
        .mem_ready       (ext_ready),
        .mem_addr        (mem_addr),
        .mem_wdata       (mem_wdata),
        .mem_wstrb       (mem_wstrb),
        .mem_rdata       (ext_rdata)
    );
    /* verilator lint_on PINCONNECTEMPTY */

    // ------------------------------------------- external port: CPU or DMA
    logic dma_sel;
    assign dma_sel = dma_busy;

    assign m_axi_awaddr  = (dma_sel ? d_awaddr : c_awaddr) & 32'h7FFF_FFFF;
    assign m_axi_awlen   = dma_sel ? d_awlen   : 8'd0;
    assign m_axi_awsize  = dma_sel ? d_awsize  : 3'd2;
    assign m_axi_awburst = dma_sel ? d_awburst : 2'b01;
    assign m_axi_awvalid = dma_sel ? d_awvalid : c_awvalid;
    assign m_axi_wdata   = dma_sel ? d_wdata   : c_wdata;
    assign m_axi_wstrb   = dma_sel ? d_wstrb   : c_wstrb;
    assign m_axi_wlast   = dma_sel ? d_wlast   : 1'b1;
    assign m_axi_wvalid  = dma_sel ? d_wvalid  : c_wvalid;
    assign m_axi_bready  = dma_sel ? d_bready  : c_bready;
    assign m_axi_araddr  = (dma_sel ? d_araddr : c_araddr) & 32'h7FFF_FFFF;
    assign m_axi_arlen   = dma_sel ? d_arlen   : 8'd0;
    assign m_axi_arsize  = dma_sel ? d_arsize  : 3'd2;
    assign m_axi_arburst = dma_sel ? d_arburst : 2'b01;
    assign m_axi_arvalid = dma_sel ? d_arvalid : c_arvalid;
    assign m_axi_rready  = dma_sel ? d_rready  : c_rready;

    assign d_awready = dma_sel && m_axi_awready;
    assign d_wready  = dma_sel && m_axi_wready;
    assign d_bvalid  = dma_sel && m_axi_bvalid;
    assign d_bresp   = m_axi_bresp;
    assign d_arready = dma_sel && m_axi_arready;
    assign d_rvalid  = dma_sel && m_axi_rvalid;
    assign d_rdata   = m_axi_rdata;
    assign d_rresp   = m_axi_rresp;
    assign d_rlast   = m_axi_rlast;

    assign c_awready = !dma_sel && m_axi_awready;
    assign c_wready  = !dma_sel && m_axi_wready;
    assign c_bvalid  = !dma_sel && m_axi_bvalid;
    assign c_arready = !dma_sel && m_axi_arready;
    assign c_rvalid  = !dma_sel && m_axi_rvalid;

endmodule
