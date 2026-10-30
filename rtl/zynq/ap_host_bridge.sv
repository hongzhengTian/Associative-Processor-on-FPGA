// Host interface of the AP SoC for a Zynq (or any system with an AXI host).
//
// Control (AXI4-Lite slave, 128 KiB):
//
//   0x0_0000  CTRL       rw  bit 0 RUN: 1 releases the SoC from reset; 0 holds
//                            it in reset and gives the program RAM to the host.
//                            Setting RUN clears DONE, TRAP, EXIT and RUN_CYCLES.
//   0x0_0004  STATUS     ro  bit 0 DONE (program wrote its exit code),
//                            bit 1 TRAP (CPU trapped), bit 2 console data
//                            waiting, bit 3 RUN
//   0x0_0008  EXIT       ro  exit code written by the program
//   0x0_000C  CONSOLE    ro  bit 8 = valid, bits 7:0 = next console byte;
//                            reading removes the byte
//   0x0_0010  RUN_CYCLES_LO  ro  cycles from RUN until DONE/TRAP
//   0x0_0014  RUN_CYCLES_HI  ro
//   0x0_0018  CLK_COUNT  ro  free-running cycle counter (clock measurement)
//   0x0_001C  ROWS       ro  AP array shape
//   0x0_0020  COLS       ro
//   0x0_0024  EXT_BASE   rw  memory address of the SoC's external window
//                            (low 12 bits ignored)
//   0x0_0028  EXT_SIZE   rw  size of the window in bytes (low 12 bits ignored)
//   0x0_002C  ID         ro  0x41500002 ("AP", version 2)
//   0x1_0000  program RAM (64 KiB), accessible while RUN = 0
//
// SoC I/O port: a write to 0x1000_0000 queues a console byte (the CPU waits
// while the queue is full), a write to 0x1000_0004 sets EXIT and DONE.
//
// External memory: SoC address offset x (address & 0x7FFF_FFFF) becomes
// EXT_BASE + x on m_mem_*. Bursts at offsets >= EXT_SIZE never reach the
// memory; they are answered here with SLVERR, which keeps a program from
// touching memory outside the buffer the host gave it. The SoC issues at
// most one read and one write burst at a time and never crosses 4 KiB.
module ap_host_bridge #(
    parameter int ROWS          = 512,
    parameter int COLS          = 64,
    parameter int CONSOLE_DEPTH = 1024      // power of two
) (
    input  logic        clk,
    input  logic        rst_n,

    // ------------------------------------------------- AXI4-Lite slave
    // (word accesses only: address bits 1:0 are ignored)
    /* verilator lint_off UNUSEDSIGNAL */
    input  logic [16:0] s_awaddr,
    input  logic        s_awvalid,
    output logic        s_awready,
    input  logic [31:0] s_wdata,
    input  logic [3:0]  s_wstrb,
    input  logic        s_wvalid,
    output logic        s_wready,
    output logic [1:0]  s_bresp,
    output logic        s_bvalid,
    input  logic        s_bready,
    input  logic [16:0] s_araddr,
    /* verilator lint_on UNUSEDSIGNAL */
    input  logic        s_arvalid,
    output logic        s_arready,
    output logic [31:0] s_rdata,
    output logic [1:0]  s_rresp,
    output logic        s_rvalid,
    input  logic        s_rready,

    // ------------------------------------------------------- SoC control
    output logic        soc_rst_n,
    output logic        host_sel,
    output logic        host_valid,
    input  logic        host_ready,
    output logic [31:0] host_addr,
    output logic [31:0] host_wdata,
    output logic [3:0]  host_wstrb,
    input  logic [31:0] host_rdata,
    input  logic        trap,

    input  logic        io_valid,
    input  logic [31:0] io_addr,
    input  logic [31:0] io_wdata,
    input  logic [3:0]  io_wstrb,
    output logic        io_ready,
    output logic [31:0] io_rdata,

    // ------------------------------------ external memory, SoC side (slave)
    input  logic [31:0] e_awaddr,
    input  logic [7:0]  e_awlen,
    input  logic [2:0]  e_awsize,
    input  logic [1:0]  e_awburst,
    input  logic        e_awvalid,
    output logic        e_awready,
    input  logic [31:0] e_wdata,
    input  logic [3:0]  e_wstrb,
    input  logic        e_wlast,
    input  logic        e_wvalid,
    output logic        e_wready,
    output logic [1:0]  e_bresp,
    output logic        e_bvalid,
    input  logic        e_bready,
    input  logic [31:0] e_araddr,
    input  logic [7:0]  e_arlen,
    input  logic [2:0]  e_arsize,
    input  logic [1:0]  e_arburst,
    input  logic        e_arvalid,
    output logic        e_arready,
    output logic [31:0] e_rdata,
    output logic [1:0]  e_rresp,
    output logic        e_rlast,
    output logic        e_rvalid,
    input  logic        e_rready,

    // ------------------------------------ external memory, system side (master)
    output logic [31:0] m_awaddr,
    output logic [7:0]  m_awlen,
    output logic [2:0]  m_awsize,
    output logic [1:0]  m_awburst,
    output logic        m_awvalid,
    input  logic        m_awready,
    output logic [31:0] m_wdata,
    output logic [3:0]  m_wstrb,
    output logic        m_wlast,
    output logic        m_wvalid,
    input  logic        m_wready,
    input  logic [1:0]  m_bresp,
    input  logic        m_bvalid,
    output logic        m_bready,
    output logic [31:0] m_araddr,
    output logic [7:0]  m_arlen,
    output logic [2:0]  m_arsize,
    output logic [1:0]  m_arburst,
    output logic        m_arvalid,
    input  logic        m_arready,
    input  logic [31:0] m_rdata,
    input  logic [1:0]  m_rresp,
    input  logic        m_rlast,
    input  logic        m_rvalid,
    output logic        m_rready
);

    localparam logic [1:0]  RESP_OKAY   = 2'b00;
    localparam logic [1:0]  RESP_SLVERR = 2'b10;
    localparam logic [31:0] ID_VALUE    = 32'h4150_0002;
    localparam int          CW          = $clog2(CONSOLE_DEPTH);

    // ============================================================ registers
    logic        run, done, trapped;
    logic [31:0] exit_code;
    logic [63:0] run_cycles;
    logic [31:0] clk_count;
    logic [31:0] ext_base, ext_size;

    assign soc_rst_n = rst_n && run;
    assign host_sel  = !run;

    // ============================================================ console
    logic [7:0]  con_mem [CONSOLE_DEPTH];
    logic [CW:0] con_wr, con_rd;
    logic        con_full, con_empty, con_push, con_pop;
    logic [7:0]  con_head;

    assign con_empty = (con_wr == con_rd);
    assign con_full  = (con_wr - con_rd) == (CW+1)'(CONSOLE_DEPTH);
    assign con_head  = con_mem[con_rd[CW-1:0]];

    // Kept out of the reset block below so that it maps to RAM.
    always_ff @(posedge clk) begin
        if (con_push) con_mem[con_wr[CW-1:0]] <= io_wdata[7:0];
    end

    // ======================================================== SoC I/O port
    logic io_write, io_is_console, io_is_exit;

    assign io_write      = io_valid && (io_wstrb != 4'b0000);
    assign io_is_console = (io_addr == 32'h1000_0000);
    assign io_is_exit    = (io_addr == 32'h1000_0004);
    assign io_ready      = io_valid && !(io_write && io_is_console && con_full);
    assign io_rdata      = 32'd0;
    assign con_push      = io_write && io_is_console && !con_full;

    // ===================================================== AXI-Lite slave
    typedef enum logic [2:0] { L_IDLE, L_WRAM, L_WRESP, L_RRAM, L_RDATA } lstate_t;

    lstate_t     lstate;
    logic        wr_go, rd_go;
    logic [13:0] l_word;                      // RAM word address
    logic [31:0] l_wdata;
    logic [3:0]  l_wstrb;
    logic [1:0]  l_resp;
    logic [31:0] l_rdata;

    // A write needs address and data together; writes go before reads.
    assign wr_go     = (lstate == L_IDLE) && s_awvalid && s_wvalid;
    assign rd_go     = (lstate == L_IDLE) && s_arvalid && !wr_go;
    assign s_awready = wr_go;
    assign s_wready  = wr_go;
    assign s_arready = rd_go;
    assign s_bvalid  = (lstate == L_WRESP);
    assign s_bresp   = l_resp;
    assign s_rvalid  = (lstate == L_RDATA);
    assign s_rresp   = l_resp;
    assign s_rdata   = l_rdata;

    assign host_valid = (lstate == L_WRAM) || (lstate == L_RRAM);
    assign host_addr  = {16'd0, l_word, 2'b00};
    assign host_wdata = l_wdata;
    assign host_wstrb = (lstate == L_WRAM) ? l_wstrb : 4'b0000;

    function automatic logic [31:0] reg_read(input logic [5:0] index);
        case (index)
            6'h00:   reg_read = {31'd0, run};
            6'h01:   reg_read = {28'd0, run, !con_empty, trapped, done};
            6'h02:   reg_read = exit_code;
            6'h03:   reg_read = {23'd0, !con_empty, con_head};
            6'h04:   reg_read = run_cycles[31:0];
            6'h05:   reg_read = run_cycles[63:32];
            6'h06:   reg_read = clk_count;
            6'h07:   reg_read = 32'(ROWS);
            6'h08:   reg_read = 32'(COLS);
            6'h09:   reg_read = ext_base;
            6'h0A:   reg_read = ext_size;
            6'h0B:   reg_read = ID_VALUE;
            default: reg_read = 32'd0;
        endcase
    endfunction

    assign con_pop = rd_go && !s_araddr[16] && (s_araddr[7:2] == 6'h03) && !con_empty;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lstate     <= L_IDLE;
            l_word     <= '0;
            l_wdata    <= '0;
            l_wstrb    <= '0;
            l_resp     <= RESP_OKAY;
            l_rdata    <= '0;
            run        <= 1'b0;
            done       <= 1'b0;
            trapped    <= 1'b0;
            exit_code  <= '0;
            run_cycles <= '0;
            clk_count  <= '0;
            ext_base   <= '0;
            ext_size   <= '0;
            con_wr     <= '0;
            con_rd     <= '0;
        end else begin
            clk_count <= clk_count + 32'd1;

            // ---------------------------------------------- SoC side events
            if (run && !done && !trapped) run_cycles <= run_cycles + 64'd1;
            if (run && trap) trapped <= 1'b1;
            if (io_valid && io_ready && io_write && io_is_exit && !done) begin
                done      <= 1'b1;
                exit_code <= io_wdata;
            end
            if (con_push) con_wr <= con_wr + (CW+1)'(1);
            if (con_pop)  con_rd <= con_rd + (CW+1)'(1);

            // -------------------------------------------- AXI-Lite slave
            case (lstate)
            L_IDLE: begin
                if (wr_go) begin
                    l_word  <= s_awaddr[15:2];
                    l_wdata <= s_wdata;
                    l_wstrb <= s_wstrb;
                    l_resp  <= RESP_OKAY;
                    if (s_awaddr[16]) begin
                        if (run) begin
                            l_resp <= RESP_SLVERR;    // RAM belongs to the SoC
                            lstate <= L_WRESP;
                        end else begin
                            lstate <= L_WRAM;
                        end
                    end else begin
                        lstate <= L_WRESP;
                        case (s_awaddr[7:2])
                            6'h00: begin
                                if (s_wdata[0] && !run) begin
                                    done       <= 1'b0;
                                    trapped    <= 1'b0;
                                    exit_code  <= '0;
                                    run_cycles <= '0;
                                    con_rd     <= con_wr;     // drop old output
                                end
                                run <= s_wdata[0];
                            end
                            6'h09:   ext_base <= {s_wdata[31:12], 12'd0};
                            6'h0A:   ext_size <= {s_wdata[31:12], 12'd0};
                            default: ;
                        endcase
                    end
                end else if (rd_go) begin
                    l_word <= s_araddr[15:2];
                    l_resp <= RESP_OKAY;
                    if (s_araddr[16]) begin
                        if (run) begin
                            l_resp  <= RESP_SLVERR;
                            l_rdata <= '0;
                            lstate  <= L_RDATA;
                        end else begin
                            lstate <= L_RRAM;
                        end
                    end else begin
                        l_rdata <= reg_read(s_araddr[7:2]);
                        lstate  <= L_RDATA;
                    end
                end
            end
            L_WRAM:  if (host_ready) lstate <= L_WRESP;
            L_RRAM:  if (host_ready) begin
                         l_rdata <= host_rdata;
                         lstate  <= L_RDATA;
                     end
            L_WRESP: if (s_bready) lstate <= L_IDLE;
            L_RDATA: if (s_rready) lstate <= L_IDLE;
            default: lstate <= L_IDLE;
            endcase
        end
    end

    // =================================== external memory: translate, guard
    logic aw_in, ar_in;
    assign aw_in = ({1'b0, e_awaddr} < {1'b0, ext_size});
    assign ar_in = ({1'b0, e_araddr} < {1'b0, ext_size});

    // ------------------------------------------------------------ writes
    typedef enum logic [1:0] { W_IDLE, W_FWD, W_DROP, W_ERR } wstate_t;
    wstate_t wstate;

    assign m_awaddr  = ext_base + e_awaddr;
    assign m_awlen   = e_awlen;
    assign m_awsize  = e_awsize;
    assign m_awburst = e_awburst;
    assign m_awvalid = (wstate == W_IDLE) && e_awvalid && aw_in;
    assign e_awready = (wstate == W_IDLE) && (aw_in ? m_awready : 1'b1);

    assign m_wdata   = e_wdata;
    assign m_wstrb   = e_wstrb;
    assign m_wlast   = e_wlast;
    assign m_wvalid  = (wstate == W_FWD) && e_wvalid;
    assign e_wready  = (wstate == W_FWD) ? m_wready : (wstate == W_DROP);

    assign e_bvalid  = (wstate == W_FWD) ? m_bvalid : (wstate == W_ERR);
    assign e_bresp   = (wstate == W_FWD) ? m_bresp  : RESP_SLVERR;
    assign m_bready  = (wstate == W_FWD) && e_bready;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wstate <= W_IDLE;
        end else begin
            case (wstate)
                W_IDLE:  if (e_awvalid && e_awready) wstate <= aw_in ? W_FWD : W_DROP;
                W_FWD:   if (m_bvalid && e_bready) wstate <= W_IDLE;
                W_DROP:  if (e_wvalid && e_wlast) wstate <= W_ERR;
                W_ERR:   if (e_bready) wstate <= W_IDLE;
                default: wstate <= W_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------- reads
    typedef enum logic [1:0] { R_IDLE, R_FWD, R_ERR } rstate_t;
    rstate_t    rstate;
    logic [8:0] r_left;

    assign m_araddr  = ext_base + e_araddr;
    assign m_arlen   = e_arlen;
    assign m_arsize  = e_arsize;
    assign m_arburst = e_arburst;
    assign m_arvalid = (rstate == R_IDLE) && e_arvalid && ar_in;
    assign e_arready = (rstate == R_IDLE) && (ar_in ? m_arready : 1'b1);

    assign e_rvalid  = (rstate == R_FWD) ? m_rvalid : (rstate == R_ERR);
    assign e_rdata   = (rstate == R_FWD) ? m_rdata  : 32'd0;
    assign e_rresp   = (rstate == R_FWD) ? m_rresp  : RESP_SLVERR;
    assign e_rlast   = (rstate == R_FWD) ? m_rlast  : (r_left == 9'd1);
    assign m_rready  = (rstate == R_FWD) && e_rready;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rstate <= R_IDLE;
            r_left <= '0;
        end else begin
            case (rstate)
                R_IDLE: if (e_arvalid && e_arready) begin
                    rstate <= ar_in ? R_FWD : R_ERR;
                    r_left <= 9'(e_arlen) + 9'd1;
                end
                R_FWD:  if (m_rvalid && e_rready && m_rlast) rstate <= R_IDLE;
                R_ERR:  if (e_rready) begin
                    r_left <= r_left - 9'd1;
                    if (r_left == 9'd1) rstate <= R_IDLE;
                end
                default: rstate <= R_IDLE;
            endcase
        end
    end

endmodule
