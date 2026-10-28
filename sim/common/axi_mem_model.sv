// AXI4 slave memory model for simulation (32-bit data, INCR bursts).
//
// One read burst and one write burst can be in progress at a time. Accesses
// beyond WORDS answer SLVERR. Bursts that break AXI rules (crossing a 4 KiB
// boundary, unsupported size or type, wrong WLAST) are counted in
// `violations`; the testbench fails the run if there are any. With +axi_stall=<percent>, every handshake
// (ARREADY, RVALID, AWREADY, WREADY, BVALID) is held off randomly, so the
// masters are exercised under back-pressure.
`timescale 1ns/1ps

module axi_mem_model #(
    parameter int WORDS = 1 << 20     // 4 MiB
) (
    input  logic        clk,
    input  logic        rst_n,

    input  logic [31:0] awaddr,
    /* verilator lint_off UNUSEDSIGNAL */
    input  logic [7:0]  awlen,          // bursts end at wlast
    /* verilator lint_on UNUSEDSIGNAL */
    input  logic [2:0]  awsize,
    input  logic [1:0]  awburst,
    input  logic        awvalid,
    output logic        awready,
    input  logic [31:0] wdata,
    input  logic [3:0]  wstrb,
    input  logic        wlast,
    input  logic        wvalid,
    output logic        wready,
    output logic [1:0]  bresp,
    output logic        bvalid,
    input  logic        bready,
    input  logic [31:0] araddr,
    input  logic [7:0]  arlen,
    input  logic [2:0]  arsize,
    input  logic [1:0]  arburst,
    input  logic        arvalid,
    output logic        arready,
    output logic [31:0] rdata,
    output logic [1:0]  rresp,
    output logic        rlast,
    output logic        rvalid,
    input  logic        rready
);

    logic [31:0] mem [WORDS];
    int unsigned stall_pct = 0;
    int unsigned violations = 0;

    initial begin
        for (int i = 0; i < WORDS; i++) mem[i] = '0;
        if ($value$plusargs("axi_stall=%d", stall_pct)) ;
    end

    function automatic logic go();
        return ($urandom % 100) >= stall_pct;
    endfunction

    function automatic logic in_range(logic [31:0] a);
        return (a >> 2) < WORDS;
    endfunction

    task automatic check_burst(string ch, logic [31:0] addr, logic [7:0] len,
                               logic [2:0] size, logic [1:0] burst);
        if (size != 3'd2 || burst != 2'b01) begin
            $display("[axi_mem_model] %s: only 32-bit INCR bursts are supported", ch);
            violations++;
        end
        if ({20'd0, addr[11:0]} + (32'(len) + 32'd1) * 32'd4 > 32'd4096) begin
            $display("[axi_mem_model] %s burst at %h, %0d beats, crosses a 4 KiB boundary",
                     ch, addr, 32'(len) + 1);
            violations++;
        end
    endtask

    // ------------------------------------------------------------------ read
    logic        r_busy;
    logic [31:0] r_addr;
    logic [8:0]  r_left;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_busy  <= 1'b0;
            arready <= 1'b0;
            rvalid  <= 1'b0;
            rlast   <= 1'b0;
            rdata   <= '0;
            rresp   <= '0;
            r_addr  <= '0;
            r_left  <= '0;
        end else begin
            arready <= !r_busy && go();
            if (arvalid && arready) begin
                check_burst("read", araddr, arlen, arsize, arburst);
                r_busy  <= 1'b1;
                r_addr  <= araddr;
                r_left  <= 9'(arlen) + 9'd1;
                arready <= 1'b0;
            end
            if (r_busy) begin
                if (!rvalid || rready) begin
                    if (r_left != 0 && go()) begin
                        rvalid <= 1'b1;
                        rdata  <= in_range(r_addr) ? mem[r_addr >> 2] : 32'hDEAD_BEEF;
                        rresp  <= in_range(r_addr) ? 2'b00 : 2'b10;
                        rlast  <= (r_left == 9'd1);
                        r_addr <= r_addr + 32'd4;
                        r_left <= r_left - 9'd1;
                    end else begin
                        rvalid <= 1'b0;
                        if (r_left == 0) r_busy <= 1'b0;
                    end
                end
            end else if (rready) begin
                rvalid <= 1'b0;
            end
        end
    end

    // ----------------------------------------------------------------- write
    logic        w_busy, w_resp;
    logic [31:0] w_addr;
    logic [8:0]  w_left;
    logic        w_err;         // error in the burst so far
    logic        err_now;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_busy  <= 1'b0;
            w_resp  <= 1'b0;
            awready <= 1'b0;
            wready  <= 1'b0;
            bvalid  <= 1'b0;
            bresp   <= '0;
            w_addr  <= '0;
            w_left  <= '0;
            w_err   <= 1'b0;
        end else begin
            awready <= !w_busy && !w_resp && go();
            if (awvalid && awready) begin
                check_burst("write", awaddr, awlen, awsize, awburst);
                w_left  <= 9'(awlen) + 9'd1;
                w_busy  <= 1'b1;
                w_addr  <= awaddr;
                w_err   <= 1'b0;
                awready <= 1'b0;
            end
            wready <= w_busy && go();
            if (w_busy && wvalid && wready) begin
                err_now = w_err || !in_range(w_addr);
                if (in_range(w_addr)) begin
                    for (int b = 0; b < 4; b++)
                        if (wstrb[b]) mem[w_addr >> 2][8*b +: 8] = wdata[8*b +: 8];
                end
                w_err  <= err_now;
                w_addr <= w_addr + 32'd4;
                w_left <= w_left - 9'd1;
                if (wlast != (w_left == 9'd1)) begin
                    $display("[axi_mem_model] WLAST does not match AWLEN");
                    violations++;
                end
                if (wlast) begin
                    w_busy <= 1'b0;
                    w_resp <= 1'b1;
                    wready <= 1'b0;
                end
            end
            if (w_resp && !bvalid && go()) begin
                bvalid <= 1'b1;
                bresp  <= w_err ? 2'b10 : 2'b00;
            end
            if (bvalid && bready) begin
                bvalid <= 1'b0;
                w_resp <= 1'b0;
            end
        end
    end

endmodule
