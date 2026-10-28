// Associative processing (AP) core.
//
// ROWS words of COLS bits, processed bit-serially and word-parallel: an
// operation on L-bit fields takes about L cycles regardless of ROWS. A per-row
// tag bit selects the rows an operation writes, and compare operations
// compute new tags, so searches and data-dependent updates run on all rows at
// once.
//
// Commands are the decoded fields of one AP instruction (see docs/isa.md).
// cmd_ready is high while idle. Every accepted command produces exactly one
// rsp_valid pulse when it has finished; rsp_wr says whether rsp_data is a
// result for rd. cmd_legal is a combinational decode of the command inputs.
//
// The xbuf_* side port gives the DMA engine direct access to the transpose
// buffer. It may only be used while no XBUF command is running.
module ap_core
#(
    parameter int ROWS = 256,   // multiple of 32, at most 65536
    parameter int COLS = 128    // at most 2048
) (
    input  logic        clk,
    input  logic        rst_n,

    input  logic        cmd_valid,
    output logic        cmd_ready,
    input  logic [2:0]  cmd_funct3,
    input  logic [6:0]  cmd_funct7,
    input  logic [31:0] cmd_rs1,
    input  logic [31:0] cmd_rs2,
    output logic        cmd_legal,

    output logic        rsp_valid,
    output logic        rsp_wr,
    output logic [31:0] rsp_data,

    input  logic        xbuf_we,
    input  logic [4:0]  xbuf_idx,
    input  logic [31:0] xbuf_wdata,
    output logic [31:0] xbuf_rdata
);

    localparam int AW     = (COLS > 1) ? $clog2(COLS) : 1;
    localparam int CHUNKS = ROWS / 32;
    localparam int RW     = $clog2(ROWS);

    initial begin
        if (ROWS % 32 != 0 || ROWS < 32 || ROWS > ap_pkg::MAX_ROWS)
            $fatal(1, "ap_core: ROWS must be a multiple of 32 in 32..65536");
        if (COLS < 1 || COLS > ap_pkg::MAX_COLS)
            $fatal(1, "ap_core: COLS must be in 1..2048");
    end

    // ================================================================ decode
    function automatic logic legal(input logic [2:0] f3, input logic [6:0] f7);
        case (f3)
            ap_pkg::GRP_VV,
            ap_pkg::GRP_VS:   legal = (f7 <= 7'(ap_pkg::ALU_LAST));
            ap_pkg::GRP_CMP:  legal = (f7[3:0] <= 4'(ap_pkg::CMP_LAST)) && (f7[6:5] != 2'd3);
            ap_pkg::GRP_TAG:  legal = (f7 <= ap_pkg::TAG_MIN);
            ap_pkg::GRP_ELEM: legal = (f7 <= ap_pkg::ELEM_WR);
            ap_pkg::GRP_XBUF: legal = (f7 <= ap_pkg::XBUF_GET);
            ap_pkg::GRP_SYS:  legal = (f7 == ap_pkg::SYS_INFO);
            default:          legal = 1'b0;   // GRP_DMA is handled outside
        endcase
    endfunction

    assign cmd_legal = legal(cmd_funct3, cmd_funct7);

    typedef enum logic [3:0] {
        K_NONE, K_ALU, K_CMP, K_MAXMIN, K_TSAVE, K_TLOAD, K_TRANGE,
        K_COUNT, K_FIRST, K_RD, K_WR, K_XPUT, K_XGET
    } kind_t;

    typedef enum logic [1:0] { S_IDLE, S_PREP, S_RUN, S_DONE } state_t;

    // ============================================================== registers
    state_t        state;
    kind_t         kind;
    logic [6:0]    alu_op;
    logic [3:0]    cmp_cond;
    logic [1:0]    cmp_mode;
    logic          use_scalar;
    logic          mm_min, mm_signed;
    logic [10:0]   d_off, a_off, b_off;
    logic [5:0]    a_len, b_len;             // 1..32
    logic [31:0]   scalar;                    // scalar operand, value, or hi
    logic [31:0]   lo;                        // TRANGE lower bound
    logic [15:0]   row;
    logic [31:0]   xmask;                     // XPUT/XGET: rows of the segment
    logic [15:0]   nsteps, step;
    logic          msb_first;
    logic [31:0]   res;
    logic          has_rd;

    logic [ROWS-1:0] tag, carry, flag;
    logic [31:0]     xbuf [32];

    // ============================================================ bit index
    logic        first, last;
    logic [4:0]  bit_i;

    assign first = (step == 16'd0);
    assign last  = (step == nsteps - 16'd1);
    assign bit_i = msb_first ? 5'(nsteps - 16'd1 - step) : step[4:0];

    // ======================================================== plane memory
    logic [11:0]     ra_full, rb_full, w_full;
    logic [AW-1:0]   ra_addr, rb_addr, w_addr;
    logic [ROWS-1:0] ra_data, rb_data, w_data, w_mask;
    logic            a_ok, b_ok, w_ok;

    assign ra_full = (state == S_PREP) ? 12'(a_off) + 12'(a_len) - 12'd1
                                       : 12'(a_off) + 12'(bit_i);
    assign rb_full = 12'(b_off) + 12'(bit_i);
    assign w_full  = 12'(d_off) + 12'(bit_i);

    assign ra_addr = AW'(ra_full);
    assign rb_addr = AW'(rb_full);
    assign w_addr  = AW'(w_full);

    // Bits outside the array read as 0 and are never written; source bits
    // beyond a field's length read as 0 (zero extension).
    assign a_ok = (ra_full < 12'(COLS)) && ((state == S_PREP) || (6'(bit_i) < a_len));
    assign b_ok = (rb_full < 12'(COLS)) && (6'(bit_i) < b_len);
    assign w_ok = (w_full  < 12'(COLS));

    ap_plane_mem #(.ROWS(ROWS), .COLS(COLS)) u_mem (
        .clk     (clk),
        .ra_addr (ra_addr),
        .ra_data (ra_data),
        .rb_addr (rb_addr),
        .rb_data (rb_data),
        .w_addr  (w_addr),
        .w_data  (w_data),
        .w_mask  (w_mask)
    );

    logic [ROWS-1:0] a_vec, b_vec;
    assign a_vec = a_ok ? ra_data : '0;
    assign b_vec = use_scalar ? {ROWS{scalar[bit_i]}} : (b_ok ? rb_data : '0);

    // ============================================================= row ALU
    logic [ROWS-1:0] alu_result, carry_d, flag_d, cond;
    logic            signed_cmp;

    assign signed_cmp = (cmp_cond == ap_pkg::CMP_LT) || (cmp_cond == ap_pkg::CMP_GE) ||
                        (cmp_cond == ap_pkg::CMP_GT) || (cmp_cond == ap_pkg::CMP_LE);

    ap_row_alu #(.ROWS(ROWS)) u_alu (
        .alu_op   (alu_op),
        .is_cmp   (kind == K_CMP),
        .cmp_cond (cmp_cond),
        .first    (first),
        .sign_bit (signed_cmp && (6'(bit_i) == a_len - 6'd1)),
        .a        (a_vec),
        .b        (b_vec),
        .carry_q  (carry),
        .flag_q   (flag),
        .result   (alu_result),
        .carry_d  (carry_d),
        .flag_d   (flag_d),
        .cond     (cond)
    );

    // ============================================== row and chunk selection
    logic [CHUNKS-1:0] seg_sel;     // 32-row segment of row[15:5]
    logic [CHUNKS-1:0] step_sel;    // 32-row chunk visited by COUNT/FIRST/TRANGE
    logic [ROWS-1:0]   row_sel;     // one-hot row
    logic [31:0]       seg_a;       // a_vec bits of the selected segment
    logic [31:0]       tag_chunk;   // tag bits of the visited chunk
    logic [31:0]       xcol;        // bit bit_i of every xbuf word

    always_comb begin
        for (int j = 0; j < CHUNKS; j++) begin
            seg_sel[j]  = (row[15:5] == 11'(j));
            step_sel[j] = (step == 16'(j));
        end
        for (int r = 0; r < ROWS; r++) row_sel[r] = (row == 16'(r));
        seg_a     = '0;
        tag_chunk = '0;
        for (int j = 0; j < CHUNKS; j++) begin
            if (seg_sel[j])  seg_a     = a_vec[j*32 +: 32];
            if (step_sel[j]) tag_chunk = tag[j*32 +: 32];
        end
        for (int k = 0; k < 32; k++) xcol[k] = xbuf[k][bit_i];
    end

    // ---------------------------------------------------------- write port
    always_comb begin
        w_data = alu_result;
        w_mask = '0;
        if (state == S_RUN) begin
            case (kind)
                K_ALU:   begin w_data = alu_result; w_mask = tag;  end
                K_TSAVE: begin w_data = tag;        w_mask = '1;   end
                K_WR:    begin w_data = {ROWS{scalar[bit_i]}}; w_mask = row_sel; end
                K_XPUT: begin
                    w_data = {CHUNKS{xcol}};
                    for (int j = 0; j < CHUNKS; j++) w_mask[j*32 +: 32] = {32{seg_sel[j]}} & xmask;
                end
                default: ;
            endcase
        end
        if (!w_ok) w_mask = '0;
    end

    // ======================================================= MAX/MIN search
    // Bits are visited MSB first. A row stays a candidate if its bit is the
    // preferred value (1 for MAX, 0 for MIN, inverted at a sign bit) or if no
    // candidate has the preferred value; that decides the result bit too.
    logic            mm_inv, mm_any;
    logic [ROWS-1:0] mm_cand;
    logic [31:0]     mm_res;

    assign mm_inv  = mm_min ^ (mm_signed & first);
    assign mm_cand = tag & (a_vec ^ {ROWS{mm_inv}});
    assign mm_any  = |mm_cand;

    always_comb begin
        mm_res        = res;
        mm_res[bit_i] = mm_any ^ mm_inv;
    end

    // ============================================================= helpers
    function automatic logic [5:0] popcount32(input logic [31:0] v);
        logic [5:0] n;
        n = '0;
        for (int k = 0; k < 32; k++) n = n + 6'(v[k]);
        popcount32 = n;
    endfunction

    function automatic logic [4:0] ctz32(input logic [31:0] v);
        logic [4:0] n;
        n = '0;
        for (int k = 31; k >= 0; k--) if (v[k]) n = 5'(k);
        ctz32 = n;
    endfunction

    function automatic logic [31:0] trange_chunk(input logic [15:0] chunk,
                                                 input logic [31:0] l,
                                                 input logic [31:0] h);
        logic [31:0] m;
        logic [31:0] r;
        for (int k = 0; k < 32; k++) begin
            r    = {11'd0, chunk[15:0], 5'(k)};
            m[k] = (r >= l) && (r < h);
        end
        trange_chunk = m;
    endfunction

    function automatic logic [31:0] sext(input logic [31:0] v, input logic [5:0] len);
        logic [31:0] o;
        for (int k = 0; k < 32; k++) o[k] = (6'(k) < len) ? v[k] : v[5'(len - 6'd1)];
        sext = o;
    endfunction

    // ======================================================== control FSM
    logic [15:0] desc_lo, desc_hi, desc_b;
    assign desc_lo = cmd_rs1[15:0];
    assign desc_hi = cmd_rs1[31:16];
    assign desc_b  = cmd_rs2[15:0];

    assign cmd_ready  = (state == S_IDLE);
    assign rsp_valid  = (state == S_DONE);
    assign rsp_wr     = has_rd;
    assign rsp_data   = res;
    assign xbuf_rdata = xbuf[xbuf_idx];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            kind       <= K_NONE;
            alu_op     <= ap_pkg::ALU_ADD;
            cmp_cond   <= ap_pkg::CMP_EQ;
            cmp_mode   <= ap_pkg::TMODE_SET;
            use_scalar <= 1'b0;
            mm_min     <= 1'b0;
            mm_signed  <= 1'b0;
            d_off      <= '0;
            a_off      <= '0; a_len <= 6'd1;
            b_off      <= '0; b_len <= 6'd1;
            scalar     <= '0;
            lo         <= '0;
            row        <= '0;
            xmask      <= '1;
            nsteps     <= 16'd1;
            step       <= '0;
            msb_first  <= 1'b0;
            res        <= '0;
            has_rd     <= 1'b0;
            tag        <= '1;
            carry      <= '0;
            flag       <= '0;
            for (int k = 0; k < 32; k++) xbuf[k] <= '0;
        end else begin
            if (xbuf_we) xbuf[xbuf_idx] <= xbuf_wdata;

            case (state)
            // ---------------------------------------------------------------
            S_IDLE: if (cmd_valid) begin
                kind       <= K_NONE;
                alu_op     <= cmd_funct7;
                cmp_cond   <= cmd_funct7[3:0];
                cmp_mode   <= cmd_funct7[6:5];
                use_scalar <= 1'b0;
                msb_first  <= 1'b0;
                step       <= '0;
                res        <= '0;
                has_rd     <= 1'b0;
                scalar     <= cmd_rs2;
                lo         <= cmd_rs1;
                row        <= desc_hi;
                xmask      <= (cmd_rs2 == 32'd0 || cmd_rs2 >= 32'd32) ? '1
                                                                       : ((32'd1 << cmd_rs2[4:0]) - 32'd1);
                state      <= S_RUN;

                // Fields: arithmetic uses rs1 = {a, d}, rs2 = b; everything
                // else names its single field in rs1[15:0], kept as both the
                // source (a) and the destination (d) descriptor.
                d_off <= desc_lo[10:0];
                a_off <= desc_lo[10:0];  a_len <= {1'b0, desc_lo[15:11]} + 6'd1;
                b_off <= desc_b[10:0];   b_len <= {1'b0, desc_b[15:11]} + 6'd1;
                nsteps <= {11'd0, desc_lo[15:11]} + 16'd1;

                if (!legal(cmd_funct3, cmd_funct7)) begin
                    state <= S_DONE;
                end else begin
                    case (cmd_funct3)
                    ap_pkg::GRP_VV, ap_pkg::GRP_VS: begin
                        kind       <= K_ALU;
                        use_scalar <= (cmd_funct3 == ap_pkg::GRP_VS);
                        a_off      <= desc_hi[10:0];
                        a_len      <= {1'b0, desc_hi[15:11]} + 6'd1;
                        if (cmd_funct7 == ap_pkg::ALU_ABS) state <= S_PREP;
                    end
                    ap_pkg::GRP_CMP: begin
                        kind       <= K_CMP;
                        use_scalar <= cmd_funct7[4];
                    end
                    ap_pkg::GRP_TAG: begin
                        case (cmd_funct7)
                            ap_pkg::TAG_ALL:  begin tag <= '1;   state <= S_DONE; end
                            ap_pkg::TAG_NONE: begin tag <= '0;   state <= S_DONE; end
                            ap_pkg::TAG_NOT:  begin tag <= ~tag; state <= S_DONE; end
                            ap_pkg::TAG_RANGE: begin
                                kind   <= K_TRANGE;
                                nsteps <= 16'(CHUNKS);
                            end
                            ap_pkg::TAG_SAVE: begin kind <= K_TSAVE; nsteps <= 16'd1; end
                            ap_pkg::TAG_LOAD: begin kind <= K_TLOAD; nsteps <= 16'd1; end
                            ap_pkg::TAG_COUNT: begin
                                kind   <= K_COUNT;
                                nsteps <= 16'(CHUNKS);
                                has_rd <= 1'b1;
                            end
                            ap_pkg::TAG_FIRST: begin
                                kind   <= K_FIRST;
                                nsteps <= 16'(CHUNKS);
                                res    <= '1;
                                has_rd <= 1'b1;
                            end
                            default: begin                  // MAXU/MINU/MAX/MIN
                                kind      <= K_MAXMIN;
                                msb_first <= 1'b1;
                                mm_min    <= cmd_funct7[0];
                                mm_signed <= (cmd_funct7 == ap_pkg::TAG_MAX) || (cmd_funct7 == ap_pkg::TAG_MIN);
                                has_rd    <= 1'b1;
                                if (tag == '0) state <= S_DONE;   // nothing to search
                            end
                        endcase
                    end
                    ap_pkg::GRP_ELEM: begin
                        kind   <= (cmd_funct7 == ap_pkg::ELEM_RD) ? K_RD : K_WR;
                        has_rd <= (cmd_funct7 == ap_pkg::ELEM_RD);
                    end
                    ap_pkg::GRP_XBUF: begin
                        case (cmd_funct7)
                            ap_pkg::XBUF_WR: begin
                                xbuf[cmd_rs1[4:0]] <= cmd_rs2;
                                state <= S_DONE;
                            end
                            ap_pkg::XBUF_RD: begin
                                res    <= xbuf[cmd_rs1[4:0]];
                                has_rd <= 1'b1;
                                state  <= S_DONE;
                            end
                            ap_pkg::XBUF_PUT: kind <= K_XPUT;
                            default: begin
                                kind <= K_XGET;
                                for (int k = 0; k < 32; k++) xbuf[k] <= '0;
                            end
                        endcase
                    end
                    ap_pkg::GRP_SYS: begin
                        case (cmd_rs1)
                            ap_pkg::INFO_ROWS:    res <= 32'(ROWS);
                            ap_pkg::INFO_COLS:    res <= 32'(COLS);
                            ap_pkg::INFO_VERSION: res <= ap_pkg::AP_VERSION;
                            default:              res <= '0;
                        endcase
                        has_rd <= 1'b1;
                        state  <= S_DONE;
                    end
                    default: state <= S_DONE;
                    endcase
                end
            end

            // ---------------------------------------------------------------
            // ABS: fetch the sign bit of every row before the bit loop.
            S_PREP: begin
                flag  <= a_vec;
                state <= S_RUN;
            end

            // ---------------------------------------------------------------
            S_RUN: begin
                step <= step + 16'd1;
                if (last) state <= S_DONE;

                case (kind)
                K_ALU: carry <= carry_d;

                K_CMP: begin
                    flag <= flag_d;
                    if (last) begin
                        case (cmp_mode)
                            ap_pkg::TMODE_AND: tag <= tag & cond;
                            ap_pkg::TMODE_OR:  tag <= tag | cond;
                            default:           tag <= cond;
                        endcase
                    end
                end

                K_MAXMIN: begin
                    if (mm_any) tag <= mm_cand;
                    res <= (last && mm_signed) ? sext(mm_res, a_len) : mm_res;
                end

                K_TLOAD: tag <= a_vec;

                K_TRANGE: begin
                    for (int j = 0; j < CHUNKS; j++)
                        if (step_sel[j]) tag[j*32 +: 32] <= trange_chunk(16'(j), lo, scalar);
                end

                K_COUNT: res <= res + 32'(popcount32(tag_chunk));

                K_FIRST: begin
                    if (tag_chunk != '0) begin
                        res   <= 32'({step[10:0], 5'd0}) + 32'(ctz32(tag_chunk));
                        state <= S_DONE;
                    end
                end

                K_RD: res[bit_i] <= (32'(row) < 32'(ROWS)) ? a_vec[row[RW-1:0]] : 1'b0;

                K_XGET: begin
                    for (int k = 0; k < 32; k++) xbuf[k][bit_i] <= seg_a[k] & xmask[k];
                end

                default: ;   // K_WR, K_XPUT, K_TSAVE only write the array
                endcase
            end

            // ---------------------------------------------------------------
            S_DONE: state <= S_IDLE;

            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
