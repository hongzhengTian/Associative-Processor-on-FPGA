// PicoRV32 co-processor interface (PCPI) adapter for the AP unit.
//
// PicoRV32 offers every instruction it does not implement on PCPI, with the
// rs1/rs2 values already read, and stalls until pcpi_ready. This adapter
// claims custom-1 instructions that the AP unit decodes as legal (others
// time out in the core and raise an illegal-instruction trap), forwards them
// as one command, keeps pcpi_wait high while the unit works, and returns the
// result for rd.
module ap_pcpi (
    input  logic        clk,
    input  logic        rst_n,

    input  logic        pcpi_valid,
    /* verilator lint_off UNUSEDSIGNAL */
    input  logic [31:0] pcpi_insn,      // rd/rs1/rs2 numbers are handled by the CPU
    /* verilator lint_on UNUSEDSIGNAL */
    input  logic [31:0] pcpi_rs1,
    input  logic [31:0] pcpi_rs2,
    output logic        pcpi_wr,
    output logic [31:0] pcpi_rd,
    output logic        pcpi_wait,
    output logic        pcpi_ready,

    output logic        cmd_valid,
    input  logic        cmd_ready,
    output logic [2:0]  cmd_funct3,
    output logic [6:0]  cmd_funct7,
    output logic [31:0] cmd_rs1,
    output logic [31:0] cmd_rs2,
    input  logic        cmd_legal,
    input  logic        rsp_valid,
    input  logic        rsp_wr,
    input  logic [31:0] rsp_data
);

    logic busy;
    logic match;

    assign cmd_funct3 = pcpi_insn[14:12];
    assign cmd_funct7 = pcpi_insn[31:25];
    assign cmd_rs1    = pcpi_rs1;
    assign cmd_rs2    = pcpi_rs2;

    assign match      = pcpi_valid && (pcpi_insn[6:0] == ap_pkg::OPCODE_CUSTOM1) && cmd_legal;
    assign cmd_valid  = match && !busy && cmd_ready;

    assign pcpi_wait  = match || busy;
    assign pcpi_ready = busy && rsp_valid;
    assign pcpi_wr    = busy && rsp_valid && rsp_wr;
    assign pcpi_rd    = rsp_data;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)          busy <= 1'b0;
        else if (cmd_valid)  busy <= 1'b1;
        else if (rsp_valid)  busy <= 1'b0;
    end

endmodule
