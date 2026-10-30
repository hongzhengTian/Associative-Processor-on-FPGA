// DMA engine between external memory (AXI4) and AP fields.
//
// Element i of a transfer is the 32-bit word at address + 4*i and belongs to
// row base + i of the array (base is a multiple of 32). The engine works in
// segments of 32 rows, using the core's transpose buffer:
//
//   LOAD   read up to 32 words (INCR bursts) into the buffer, then XPUT
//   STORE  XGET, then write the buffer out (INCR bursts)
//
// Bursts are split at 4 KiB boundaries as AXI requires. Commands:
//
//   DMA_CFG    address = rs1, element count = rs2
//   DMA_LOAD   rs1 = {base row, field}; rd = status (0 ok, 1 bus error)
//   DMA_STORE  rs1 = {base row, field}; rd = status
//
// Address and count stay set across transfers. The data bus is 32 bits wide.
module ap_dma #(
    parameter int ADDR_W = 32
) (
    input  logic              clk,
    input  logic              rst_n,

    // DMA_* commands
    input  logic              cmd_valid,
    output logic              cmd_ready,
    input  logic [6:0]        cmd_funct7,
    input  logic [31:0]       cmd_rs1,
    input  logic [31:0]       cmd_rs2,
    output logic              rsp_valid,
    output logic              rsp_wr,
    output logic [31:0]       rsp_data,
    output logic              busy,

    // commands to the AP core while busy (XPUT / XGET)
    output logic              core_cmd_valid,
    input  logic              core_cmd_ready,
    output logic [2:0]        core_cmd_funct3,
    output logic [6:0]        core_cmd_funct7,
    output logic [31:0]       core_cmd_rs1,
    output logic [31:0]       core_cmd_rs2,
    input  logic              core_rsp_valid,

    // transpose buffer side port
    output logic              xbuf_we,
    output logic [4:0]        xbuf_idx,
    output logic [31:0]       xbuf_wdata,
    input  logic [31:0]       xbuf_rdata,

    // AXI4 master (single ID, 32-bit data)
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
    /* verilator lint_off UNUSEDSIGNAL */
    input  logic              m_axi_rlast,   // bursts are tracked by beat count
    /* verilator lint_on UNUSEDSIGNAL */
    input  logic              m_axi_rvalid,
    output logic              m_axi_rready
);

    typedef enum logic [3:0] {
        D_IDLE,
        D_SEG,          // start a 32-row segment
        D_AR, D_R,      // load: read burst into the buffer
        D_PUT,          // load: buffer -> array
        D_GET,          // store: array -> buffer
        D_AW, D_W, D_B, // store: write burst from the buffer
        D_CORE_WAIT,    // wait for the core command of this segment
        D_DONE
    } dstate_t;

    dstate_t     state;
    logic        is_store;
    logic [31:0] cfg_addr, cfg_count;
    logic [31:0] addr;         // address of the next word
    logic [31:0] remaining;    // elements not yet transferred
    logic [10:0] seg;          // current segment (row / 32)
    logic [15:0] field;
    logic [5:0]  seg_n;        // elements in the current segment, 1..32
    logic [5:0]  k;            // next word of the segment
    logic [8:0]  beats;        // beats left in the current burst
    logic        err;

    // Words that fit in a burst starting at addr: the rest of the segment, but
    // never across a 4 KiB boundary.
    logic [10:0] to_4k;
    logic [5:0]  seg_left;
    logic [5:0]  burst_len;
    assign to_4k     = 11'((13'd4096 - {1'b0, addr[11:0]}) >> 2);
    assign seg_left  = seg_n - k;
    assign burst_len = ({5'd0, seg_left} < to_4k) ? seg_left : to_4k[5:0];

    assign cmd_ready = (state == D_IDLE);
    assign busy      = (state != D_IDLE);
    assign rsp_valid = (state == D_DONE);
    assign rsp_wr    = 1'b1;
    assign rsp_data  = {31'd0, err};

    // core commands
    assign core_cmd_valid  = ((state == D_PUT) || (state == D_GET)) && core_cmd_ready;
    assign core_cmd_funct3 = ap_pkg::GRP_XBUF;
    assign core_cmd_funct7 = (state == D_GET) ? ap_pkg::XBUF_GET : ap_pkg::XBUF_PUT;
    assign core_cmd_rs1    = {seg, 5'd0, field};
    assign core_cmd_rs2    = 32'(seg_n);

    // transpose buffer
    assign xbuf_we    = (state == D_R) && m_axi_rvalid;
    assign xbuf_idx   = k[4:0];
    assign xbuf_wdata = m_axi_rdata;

    // AXI
    assign m_axi_araddr  = ADDR_W'({addr[31:2], 2'b00});
    assign m_axi_arlen   = {2'b00, burst_len - 6'd1};
    assign m_axi_arsize  = 3'd2;
    assign m_axi_arburst = 2'b01;
    assign m_axi_arvalid = (state == D_AR);
    assign m_axi_rready  = (state == D_R);

    assign m_axi_awaddr  = ADDR_W'({addr[31:2], 2'b00});
    assign m_axi_awlen   = {2'b00, burst_len - 6'd1};
    assign m_axi_awsize  = 3'd2;
    assign m_axi_awburst = 2'b01;
    assign m_axi_awvalid = (state == D_AW);
    assign m_axi_wdata   = xbuf_rdata;
    assign m_axi_wstrb   = 4'hF;
    assign m_axi_wlast   = (beats == 9'd1);
    assign m_axi_wvalid  = (state == D_W);
    assign m_axi_bready  = (state == D_B);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= D_IDLE;
            is_store  <= 1'b0;
            cfg_addr  <= '0;
            cfg_count <= '0;
            addr      <= '0;
            remaining <= '0;
            seg       <= '0;
            field     <= '0;
            seg_n     <= 6'd1;
            k         <= '0;
            beats     <= '0;
            err       <= 1'b0;
        end else begin
            case (state)
            D_IDLE: if (cmd_valid) begin
                err <= 1'b0;
                if (cmd_funct7 == ap_pkg::DMA_CFG) begin
                    cfg_addr  <= cmd_rs1;
                    cfg_count <= cmd_rs2;
                    state     <= D_DONE;
                end else begin
                    is_store  <= (cmd_funct7 == ap_pkg::DMA_STORE);
                    addr      <= cfg_addr;
                    remaining <= cfg_count;
                    seg       <= cmd_rs1[31:21];
                    field     <= cmd_rs1[15:0];
                    state     <= (cfg_count == 32'd0) ? D_DONE : D_SEG;
                end
            end

            D_SEG: begin
                seg_n <= (remaining >= 32'd32) ? 6'd32 : 6'(remaining);
                k     <= '0;
                state <= is_store ? D_GET : D_AR;
            end

            // ------------------------------------------------------- load
            D_AR: if (m_axi_arready) begin
                beats <= 9'(burst_len);
                state <= D_R;
            end

            D_R: if (m_axi_rvalid) begin
                if (m_axi_rresp != 2'b00) err <= 1'b1;
                k     <= k + 6'd1;
                addr  <= addr + 32'd4;
                beats <= beats - 9'd1;
                if (beats == 9'd1) state <= (k + 6'd1 == seg_n) ? D_PUT : D_AR;
            end

            D_PUT: if (core_cmd_ready) state <= D_CORE_WAIT;

            // ------------------------------------------------------ store
            D_GET: if (core_cmd_ready) state <= D_CORE_WAIT;

            D_AW: if (m_axi_awready) begin
                beats <= 9'(burst_len);
                state <= D_W;
            end

            D_W: if (m_axi_wready) begin
                k     <= k + 6'd1;
                addr  <= addr + 32'd4;
                beats <= beats - 9'd1;
                if (beats == 9'd1) state <= D_B;
            end

            D_B: if (m_axi_bvalid) begin
                if (m_axi_bresp != 2'b00) err <= 1'b1;
                if (k == seg_n) begin
                    remaining <= remaining - 32'(seg_n);
                    seg       <= seg + 11'd1;
                    state     <= (remaining == 32'(seg_n)) ? D_DONE : D_SEG;
                end else begin
                    state <= D_AW;
                end
            end

            // ------------------------------------------------------ common
            D_CORE_WAIT: if (core_rsp_valid) begin
                if (is_store) begin
                    state <= D_AW;                    // buffer filled, write it
                end else begin
                    remaining <= remaining - 32'(seg_n);
                    seg       <= seg + 11'd1;
                    state     <= (remaining == 32'(seg_n)) ? D_DONE : D_SEG;
                end
            end

            D_DONE: state <= D_IDLE;

            default: state <= D_IDLE;
            endcase
        end
    end

endmodule
