// Testbench of the Zynq top level (rtl/zynq/ap_zynq.v): plays the role of
// the ARM host program over AXI4-Lite and of the DDR behind the HP0 port.
//
// It checks the host interface (ID, shape, program RAM access rules), loads a
// program through the control port, runs it with the external window placed
// in the middle of the memory model, prints the console output it drains
// from the bridge, and passes if the program exits with 0. The ISA test's
// DMA error case lands outside the window, so it also checks the guard.
//
// Plusargs: +firmware=<hex> +timeout=<cycles> +axi_stall=<percent>
`timescale 1ns/1ps

module tb_ap_zynq;

    parameter int ROWS = 512;
    parameter int COLS = 64;

    localparam logic [31:0] EXT_BASE = 32'h0040_0000;   // 4 MiB into the model
    localparam logic [31:0] EXT_SIZE = 32'h0040_0000;   // 4 MiB window

    localparam logic [16:0] R_CTRL = 17'h0_0000, R_STATUS = 17'h0_0004, R_EXIT = 17'h0_0008,
                            R_CONSOLE = 17'h0_000C, R_CYC_LO = 17'h0_0010, R_ROWS = 17'h0_001C,
                            R_COLS = 17'h0_0020, R_EXT_BASE = 17'h0_0024, R_EXT_SIZE = 17'h0_0028,
                            R_ID = 17'h0_002C, R_RAM = 17'h1_0000;

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    always #5 clk = ~clk;

    // ------------------------------------------------------------ AXI-Lite
    logic [16:0] awaddr = '0, araddr = '0;
    logic        awvalid = 1'b0, wvalid = 1'b0, bready = 1'b0, arvalid = 1'b0, rready = 1'b0;
    logic [31:0] wdata = '0;
    logic [3:0]  wstrb = '0;
    logic        awready, wready, bvalid, arready, rvalid;
    logic [1:0]  bresp, rresp;
    logic [31:0] rdata;

    // ------------------------------------------------------------- memory
    logic [31:0] m_awaddr, m_wdata, m_araddr, m_rdata;
    logic [7:0]  m_awlen, m_arlen;
    logic [2:0]  m_awsize, m_arsize;
    logic [1:0]  m_awburst, m_arburst, m_bresp, m_rresp;
    logic [3:0]  m_wstrb;
    logic        m_awvalid, m_awready, m_wlast, m_wvalid, m_wready, m_bvalid, m_bready;
    logic        m_arvalid, m_arready, m_rlast, m_rvalid, m_rready;

    /* verilator lint_off PINCONNECTEMPTY */
    ap_zynq #(.ROWS(ROWS), .COLS(COLS)) dut (
        .aclk               (clk),
        .aresetn            (rst_n),
        .s_axi_ctrl_awaddr  (awaddr),
        .s_axi_ctrl_awvalid (awvalid),
        .s_axi_ctrl_awready (awready),
        .s_axi_ctrl_wdata   (wdata),
        .s_axi_ctrl_wstrb   (wstrb),
        .s_axi_ctrl_wvalid  (wvalid),
        .s_axi_ctrl_wready  (wready),
        .s_axi_ctrl_bresp   (bresp),
        .s_axi_ctrl_bvalid  (bvalid),
        .s_axi_ctrl_bready  (bready),
        .s_axi_ctrl_araddr  (araddr),
        .s_axi_ctrl_arvalid (arvalid),
        .s_axi_ctrl_arready (arready),
        .s_axi_ctrl_rdata   (rdata),
        .s_axi_ctrl_rresp   (rresp),
        .s_axi_ctrl_rvalid  (rvalid),
        .s_axi_ctrl_rready  (rready),
        .m_axi_mem_awaddr   (m_awaddr),
        .m_axi_mem_awlen    (m_awlen),
        .m_axi_mem_awsize   (m_awsize),
        .m_axi_mem_awburst  (m_awburst),
        .m_axi_mem_awcache  (),
        .m_axi_mem_awprot   (),
        .m_axi_mem_awvalid  (m_awvalid),
        .m_axi_mem_awready  (m_awready),
        .m_axi_mem_wdata    (m_wdata),
        .m_axi_mem_wstrb    (m_wstrb),
        .m_axi_mem_wlast    (m_wlast),
        .m_axi_mem_wvalid   (m_wvalid),
        .m_axi_mem_wready   (m_wready),
        .m_axi_mem_bresp    (m_bresp),
        .m_axi_mem_bvalid   (m_bvalid),
        .m_axi_mem_bready   (m_bready),
        .m_axi_mem_araddr   (m_araddr),
        .m_axi_mem_arlen    (m_arlen),
        .m_axi_mem_arsize   (m_arsize),
        .m_axi_mem_arburst  (m_arburst),
        .m_axi_mem_arcache  (),
        .m_axi_mem_arprot   (),
        .m_axi_mem_arvalid  (m_arvalid),
        .m_axi_mem_arready  (m_arready),
        .m_axi_mem_rdata    (m_rdata),
        .m_axi_mem_rresp    (m_rresp),
        .m_axi_mem_rlast    (m_rlast),
        .m_axi_mem_rvalid   (m_rvalid),
        .m_axi_mem_rready   (m_rready)
    );
    /* verilator lint_on PINCONNECTEMPTY */

    axi_mem_model #(.WORDS(1 << 21)) u_mem (          // 8 MiB
        .clk     (clk),
        .rst_n   (rst_n),
        .awaddr  (m_awaddr),
        .awlen   (m_awlen),
        .awsize  (m_awsize),
        .awburst (m_awburst),
        .awvalid (m_awvalid),
        .awready (m_awready),
        .wdata   (m_wdata),
        .wstrb   (m_wstrb),
        .wlast   (m_wlast),
        .wvalid  (m_wvalid),
        .wready  (m_wready),
        .bresp   (m_bresp),
        .bvalid  (m_bvalid),
        .bready  (m_bready),
        .araddr  (m_araddr),
        .arlen   (m_arlen),
        .arsize  (m_arsize),
        .arburst (m_arburst),
        .arvalid (m_arvalid),
        .arready (m_arready),
        .rdata   (m_rdata),
        .rresp   (m_rresp),
        .rlast   (m_rlast),
        .rvalid  (m_rvalid),
        .rready  (m_rready)
    );

    // Every access the SoC makes must stay inside the window.
    always @(posedge clk) begin
        if (m_awvalid && (m_awaddr < EXT_BASE || m_awaddr >= EXT_BASE + EXT_SIZE))
            $fatal(1, "write outside the external window: %h", m_awaddr);
        if (m_arvalid && (m_araddr < EXT_BASE || m_araddr >= EXT_BASE + EXT_SIZE))
            $fatal(1, "read outside the external window: %h", m_araddr);
    end

    // ---------------------------------------------------- host operations
    int unsigned errors = 0;

    task automatic axil_write(input logic [16:0] a, input logic [31:0] d, output logic [1:0] resp);
        @(negedge clk);
        awaddr = a; awvalid = 1'b1; wdata = d; wstrb = 4'hF; wvalid = 1'b1;
        do @(posedge clk); while (!(awready && wready));
        @(negedge clk);
        awvalid = 1'b0; wvalid = 1'b0; bready = 1'b1;
        while (!bvalid) @(negedge clk);
        resp = bresp;
        @(posedge clk);
        @(negedge clk);
        bready = 1'b0;
    endtask

    task automatic axil_read(input logic [16:0] a, output logic [31:0] d, output logic [1:0] resp);
        @(negedge clk);
        araddr = a; arvalid = 1'b1;
        do @(posedge clk); while (!arready);
        @(negedge clk);
        arvalid = 1'b0; rready = 1'b1;
        while (!rvalid) @(negedge clk);
        d = rdata;
        resp = rresp;
        @(posedge clk);
        @(negedge clk);
        rready = 1'b0;
    endtask

    task automatic wr(input logic [16:0] a, input logic [31:0] d);
        logic [1:0] resp;
        axil_write(a, d, resp);
        if (resp != 2'b00) begin
            $display("ERROR: write %h returned %b", a, resp);
            errors++;
        end
    endtask

    function automatic void expect_eq(string what, logic [31:0] got, logic [31:0] exp);
        if (got !== exp) begin
            $display("ERROR: %s = %h, expected %h", what, got, exp);
            errors++;
        end
    endfunction

    // ------------------------------------------------------------- main
    logic [31:0] image [16384];

    initial begin
        string           fw;
        longint unsigned timeout = 100_000_000;
        logic [31:0]     d, status;
        logic [1:0]      resp;
        int              last;

        if (!$value$plusargs("firmware=%s", fw)) $fatal(1, "+firmware=<hex> missing");
        if ($value$plusargs("timeout=%d", timeout)) ;
        for (int i = 0; i < 16384; i++) image[i] = '0;
        $readmemh(fw, image);
        last = 0;
        for (int i = 0; i < 16384; i++) if (image[i] != 0) last = i;

        repeat (5) @(posedge clk);
        rst_n = 1'b1;

        // ------------------------------------------- interface checks
        axil_read(R_ID, d, resp);    expect_eq("ID", d, 32'h4150_0002);
        axil_read(R_ROWS, d, resp);  expect_eq("ROWS", d, 32'(ROWS));
        axil_read(R_COLS, d, resp);  expect_eq("COLS", d, 32'(COLS));

        // --------------------------------------------- load the program
        wr(R_CTRL, 0);
        for (int i = 0; i <= last; i++) wr(R_RAM + 17'(4 * i), image[i]);
        for (int i = 0; i <= last; i += 97) begin
            axil_read(R_RAM + 17'(4 * i), d, resp);
            expect_eq($sformatf("RAM[%0d]", i), d, image[i]);
        end
        wr(R_EXT_BASE, EXT_BASE);
        wr(R_EXT_SIZE, EXT_SIZE);

        // ------------------------------------------------------- run
        wr(R_CTRL, 1);
        axil_write(R_RAM, 32'hDEAD_BEEF, resp);
        expect_eq("RAM write while running: response", 32'(resp), 32'd2);

        for (longint unsigned n = 0; ; n++) begin
            axil_read(R_CONSOLE, d, resp);
            if (d[8]) $write("%c", d[7:0]);
            else begin
                axil_read(R_STATUS, status, resp);
                if (status[1:0] != 2'b00) begin
                    // drain what is left, then stop
                    do begin
                        axil_read(R_CONSOLE, d, resp);
                        if (d[8]) $write("%c", d[7:0]);
                    end while (d[8]);
                    break;
                end
            end
            if (n > timeout / 20) begin
                $display("\n[tb_ap_zynq] timeout");
                errors++;
                break;
            end
        end

        axil_read(R_STATUS, status, resp);
        axil_read(R_EXIT, d, resp);
        $display("\n[tb_ap_zynq] status %b, exit code %0d", status[3:0], d);
        if (status[1]) begin
            $display("ERROR: CPU trap");
            errors++;
        end
        expect_eq("exit code", d, 32'd0);
        axil_read(R_CYC_LO, d, resp);
        $display("[tb_ap_zynq] %0d cycles", d);
        wr(R_CTRL, 0);

        if (u_mem.violations != 0) begin
            $display("ERROR: %0d AXI protocol violations", u_mem.violations);
            errors++;
        end
        if (errors == 0) $display("PASS");
        else             $display("FAIL");
        $finish;
    end

endmodule
