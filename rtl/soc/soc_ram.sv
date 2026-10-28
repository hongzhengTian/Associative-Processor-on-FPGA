// On-chip program and data RAM with the PicoRV32 native memory handshake.
//
// One-cycle latency: a request seen with valid is answered with ready on the
// next cycle. Contents come from INIT_FILE (hex, one 32-bit word per line)
// when given; simulations can also name the file with +firmware=<path>.
module soc_ram #(
    parameter int    WORDS     = 16384,
    parameter        INIT_FILE = ""
) (
    input  logic        clk,
    input  logic        valid,
    output logic        ready,
    /* verilator lint_off UNUSEDSIGNAL */
    input  logic [31:0] addr,           // byte address, word aligned
    /* verilator lint_on UNUSEDSIGNAL */
    input  logic [31:0] wdata,
    input  logic [3:0]  wstrb,
    output logic [31:0] rdata
);

    localparam int AW = $clog2(WORDS);

    logic [31:0] mem [WORDS];

    initial begin
        if (INIT_FILE != "") $readmemh(INIT_FILE, mem);
`ifndef SYNTHESIS
        begin
            string fw;
            if ($value$plusargs("firmware=%s", fw)) $readmemh(fw, mem);
        end
`endif
    end

    logic [AW-1:0] idx;
    assign idx = addr[AW+1:2];

    always_ff @(posedge clk) begin
        ready <= 1'b0;
        if (valid && !ready) begin
            ready <= 1'b1;
            rdata <= mem[idx];
            if (wstrb[0]) mem[idx][ 7: 0] <= wdata[ 7: 0];
            if (wstrb[1]) mem[idx][15: 8] <= wdata[15: 8];
            if (wstrb[2]) mem[idx][23:16] <= wdata[23:16];
            if (wstrb[3]) mem[idx][31:24] <= wdata[31:24];
        end
    end

endmodule
