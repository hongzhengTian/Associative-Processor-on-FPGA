// System-level testbench: runs a program on ap_soc.
//
// I/O registers (CPU addresses):
//   0x1000_0000  write: print the low byte as a character
//   0x1000_0004  write: end the simulation with this exit code (0 = pass)
//
// External memory is an AXI model of 4 MiB at 0x8000_0000.
//
// Plusargs: +firmware=<hex> (read by the RAM) +timeout=<cycles>
//           +axi_stall=<percent> (AXI back-pressure) +trace_io
`timescale 1ns/1ps

module tb_soc;

    parameter int ROWS = 256;
    parameter int COLS = 128;

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    always #5 clk = ~clk;

    logic        trap;
    logic        io_valid, io_ready;
    logic [31:0] io_addr, io_wdata, io_rdata;
    logic [3:0]  io_wstrb;

    logic [31:0] awaddr, wdata, araddr, rdata;
    logic [7:0]  awlen, arlen;
    logic [2:0]  awsize, arsize;
    logic [1:0]  awburst, arburst, bresp, rresp;
    logic [3:0]  wstrb;
    logic        awvalid, awready, wlast, wvalid, wready, bvalid, bready;
    logic        arvalid, arready, rlast, rvalid, rready;

    ap_soc #(.ROWS(ROWS), .COLS(COLS)) dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .trap          (trap),
        .io_valid      (io_valid),
        .io_addr       (io_addr),
        .io_wdata      (io_wdata),
        .io_wstrb      (io_wstrb),
        .io_ready      (io_ready),
        .io_rdata      (io_rdata),
        .m_axi_awaddr  (awaddr),
        .m_axi_awlen   (awlen),
        .m_axi_awsize  (awsize),
        .m_axi_awburst (awburst),
        .m_axi_awvalid (awvalid),
        .m_axi_awready (awready),
        .m_axi_wdata   (wdata),
        .m_axi_wstrb   (wstrb),
        .m_axi_wlast   (wlast),
        .m_axi_wvalid  (wvalid),
        .m_axi_wready  (wready),
        .m_axi_bresp   (bresp),
        .m_axi_bvalid  (bvalid),
        .m_axi_bready  (bready),
        .m_axi_araddr  (araddr),
        .m_axi_arlen   (arlen),
        .m_axi_arsize  (arsize),
        .m_axi_arburst (arburst),
        .m_axi_arvalid (arvalid),
        .m_axi_arready (arready),
        .m_axi_rdata   (rdata),
        .m_axi_rresp   (rresp),
        .m_axi_rlast   (rlast),
        .m_axi_rvalid  (rvalid),
        .m_axi_rready  (rready)
    );

    axi_mem_model #(.WORDS(1 << 20)) u_mem (
        .clk     (clk),
        .rst_n   (rst_n),
        .awaddr  (awaddr),
        .awlen   (awlen),
        .awsize  (awsize),
        .awburst (awburst),
        .awvalid (awvalid),
        .awready (awready),
        .wdata   (wdata),
        .wstrb   (wstrb),
        .wlast   (wlast),
        .wvalid  (wvalid),
        .wready  (wready),
        .bresp   (bresp),
        .bvalid  (bvalid),
        .bready  (bready),
        .araddr  (araddr),
        .arlen   (arlen),
        .arsize  (arsize),
        .arburst (arburst),
        .arvalid (arvalid),
        .arready (arready),
        .rdata   (rdata),
        .rresp   (rresp),
        .rlast   (rlast),
        .rvalid  (rvalid),
        .rready  (rready)
    );

    // ------------------------------------------------------------------ I/O
    longint unsigned cycle = 0;
    always @(posedge clk) cycle <= cycle + 1;

    assign io_ready = io_valid;
    assign io_rdata = 32'd0;

    always @(posedge clk) begin
        if (rst_n && io_valid && io_wstrb != 4'b0000) begin
            case (io_addr)
                32'h1000_0000: $write("%c", io_wdata[7:0]);
                32'h1000_0004: begin
                    $display("\n[tb_soc] exit code %0d after %0d cycles", io_wdata, cycle);
                    if (u_mem.violations != 0)
                        $display("[tb_soc] %0d AXI protocol violations", u_mem.violations);
                    if (io_wdata == 32'd0 && u_mem.violations == 0) $display("PASS");
                    else                                            $display("FAIL");
                    $finish;
                end
                default: $display("[tb_soc] write to unknown I/O address %h", io_addr);
            endcase
        end
    end

    // ------------------------------------------------------- run control
    initial begin
        longint unsigned timeout = 50_000_000;
        if ($value$plusargs("timeout=%d", timeout)) ;
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        while (cycle < timeout && !trap) @(posedge clk);
        if (trap) $display("\n[tb_soc] CPU trapped at cycle %0d", cycle);
        else      $display("\n[tb_soc] timeout after %0d cycles", cycle);
        $display("FAIL");
        $finish;
    end

endmodule
