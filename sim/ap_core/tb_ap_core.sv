// Self-checking random test of ap_core against a word-level reference model.
//
// The model keeps the array row by row (not in bit planes) and implements
// every instruction directly from docs/isa.md. After each command the test
// compares the response and the complete state: every array bit, the tag and
// the transpose buffer.
//
// Plusargs: +seed=<n> +ncmds=<n>
`timescale 1ns/1ps

module tb_ap_core;

    parameter int ROWS = 64;
    parameter int COLS = 48;

    localparam int CHUNKS = ROWS / 32;

    // ------------------------------------------------------------------ DUT
    logic        clk = 1'b0;
    logic        rst_n = 1'b0;
    logic        cmd_valid = 1'b0;
    logic        cmd_ready;
    logic [2:0]  cmd_funct3 = '0;
    logic [6:0]  cmd_funct7 = '0;
    logic [31:0] cmd_rs1 = '0, cmd_rs2 = '0;
    logic        cmd_legal;
    logic        rsp_valid, rsp_wr;
    logic [31:0] rsp_data;
    logic [31:0] xbuf_rdata;

    always #5 clk = ~clk;

    ap_core #(.ROWS(ROWS), .COLS(COLS)) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .cmd_valid  (cmd_valid),
        .cmd_ready  (cmd_ready),
        .cmd_funct3 (cmd_funct3),
        .cmd_funct7 (cmd_funct7),
        .cmd_rs1    (cmd_rs1),
        .cmd_rs2    (cmd_rs2),
        .cmd_legal  (cmd_legal),
        .rsp_valid  (rsp_valid),
        .rsp_wr     (rsp_wr),
        .rsp_data   (rsp_data),
        .xbuf_we    (1'b0),
        .xbuf_idx   (5'd0),
        .xbuf_wdata (32'd0),
        .xbuf_rdata (xbuf_rdata)
    );

    // Flat view of the DUT array, row-major like the model.
    logic [COLS-1:0] dut_rows [ROWS];
    for (genvar r = 0; r < ROWS; r++) begin : g_r
        for (genvar c = 0; c < COLS; c++) begin : g_c
            assign dut_rows[r][c] = dut.u_mem.g_row[r].bits[c];
        end
    end

    // ---------------------------------------------------------------- model
    logic [COLS-1:0] m_rows [ROWS];
    logic [ROWS-1:0] m_tag;
    logic [31:0]     m_xbuf [32];

    function automatic logic [31:0] lenmask(int len);
        return (len >= 32) ? 32'hFFFF_FFFF : ((32'd1 << len) - 32'd1);
    endfunction

    function automatic logic [31:0] get_field(int r, int off, int len);
        logic [31:0] v = '0;
        if (r >= ROWS) return '0;
        for (int i = 0; i < len; i++)
            if (off + i < COLS) v[i] = m_rows[r][off + i];
        return v;
    endfunction

    task automatic set_field(int r, int off, int len, logic [31:0] v);
        if (r >= ROWS) return;
        for (int i = 0; i < len; i++)
            if (off + i < COLS) m_rows[r][off + i] = v[i];
    endtask

    function automatic logic [31:0] sext(logic [31:0] v, int len);
        return (len >= 32 || !v[len-1]) ? v : (v | ~lenmask(len));
    endfunction

    function automatic int f_off(logic [15:0] d); return int'(d[10:0]); endfunction
    function automatic int f_len(logic [15:0] d); return int'(d[15:11]) + 1; endfunction

    function automatic logic model_legal(logic [2:0] f3, logic [6:0] f7);
        case (f3)
            3'd0, 3'd1: return f7 <= 7'd10;
            3'd2:       return f7[3:0] <= 4'd9 && f7[6:5] != 2'd3;
            3'd3:       return f7 <= 7'd11;
            3'd4:       return f7 <= 7'd1;
            3'd5:       return f7 <= 7'd3;
            3'd7:       return f7 == 7'd0;
            default:    return 1'b0;
        endcase
    endfunction

    // Executes one command on the model; returns {has_rd, value}.
    task automatic model_exec(input logic [2:0] f3, input logic [6:0] f7,
                              input logic [31:0] rs1, input logic [31:0] rs2,
                              output logic has_rd, output logic [31:0] value);
        has_rd = 1'b0;
        value  = '0;
        if (!model_legal(f3, f7)) return;

        case (f3)
        3'd0, 3'd1: begin                                   // arithmetic
            int dof = f_off(rs1[15:0]), L = f_len(rs1[15:0]);
            int aof = f_off(rs1[31:16]), al = f_len(rs1[31:16]);
            int bof = f_off(rs2[15:0]),  bl = f_len(rs2[15:0]);
            logic [31:0] res [ROWS];
            for (int r = 0; r < ROWS; r++) begin
                logic [31:0] a = get_field(r, aof, al);
                logic [31:0] b = (f3 == 3'd1) ? rs2 : get_field(r, bof, bl);
                logic [31:0] v;
                logic sgn = a[al-1];
                case (f7)
                    7'd0:  v = a + b;
                    7'd1:  v = a - b;
                    7'd2:  v = a & b;
                    7'd3:  v = a | b;
                    7'd4:  v = a ^ b;
                    7'd5:  v = a;
                    7'd6:  v = ~a;
                    7'd7:  v = -a;
                    7'd8:  v = sgn ? -a : a;
                    7'd9:  v = b;
                    default: v = b - a;
                endcase
                res[r] = v & lenmask(L);
            end
            for (int r = 0; r < ROWS; r++)
                if (m_tag[r]) set_field(r, dof, L, res[r]);
        end

        3'd2: begin                                         // compare
            int aof = f_off(rs1[15:0]), L = f_len(rs1[15:0]);
            int bof = f_off(rs2[15:0]), bl = f_len(rs2[15:0]);
            logic [ROWS-1:0] cond;
            for (int r = 0; r < ROWS; r++) begin
                logic [31:0] a = get_field(r, aof, L);
                logic [31:0] b = (f7[4] ? rs2 : get_field(r, bof, bl)) & lenmask(L);
                logic [31:0] xa = sext(a, L), xb = sext(b, L);
                logic signed [31:0] sa = $signed(xa), sb = $signed(xb);
                logic lt_s, gt_s;
                lt_s = sa < sb;
                gt_s = sa > sb;
                case (f7[3:0])
                    4'd0: cond[r] = (a == b);
                    4'd1: cond[r] = (a != b);
                    4'd2: cond[r] = (a <  b);
                    4'd3: cond[r] = lt_s;
                    4'd4: cond[r] = (a >= b);
                    4'd5: cond[r] = !lt_s;
                    4'd6: cond[r] = (a >  b);
                    4'd7: cond[r] = gt_s;
                    4'd8: cond[r] = (a <= b);
                    default: cond[r] = !gt_s;
                endcase
            end
            case (f7[6:5])
                2'd1:    m_tag = m_tag & cond;
                2'd2:    m_tag = m_tag | cond;
                default: m_tag = cond;
            endcase
        end

        3'd3: begin                                         // tag
            int off = f_off(rs1[15:0]), L = f_len(rs1[15:0]);
            case (f7)
                7'd0: m_tag = '1;
                7'd1: m_tag = '0;
                7'd2: m_tag = ~m_tag;
                7'd3: for (int r = 0; r < ROWS; r++)
                          m_tag[r] = (32'(r) >= rs1) && (32'(r) < rs2);
                7'd4: if (off < COLS) for (int r = 0; r < ROWS; r++) m_rows[r][off] = m_tag[r];
                7'd5: for (int r = 0; r < ROWS; r++) m_tag[r] = (off < COLS) ? m_rows[r][off] : 1'b0;
                7'd6: begin
                    has_rd = 1'b1;
                    for (int r = 0; r < ROWS; r++) value += 32'(m_tag[r]);
                end
                7'd7: begin
                    has_rd = 1'b1;
                    value  = '1;
                    for (int r = ROWS - 1; r >= 0; r--) if (m_tag[r]) value = 32'(r);
                end
                default: begin                              // MAXU MINU MAX MIN
                    logic is_min = f7[0], is_signed = (f7 >= 7'd10);
                    logic found = 1'b0;
                    logic signed [32:0] best = '0;
                    has_rd = 1'b1;
                    for (int r = 0; r < ROWS; r++) if (m_tag[r]) begin
                        logic [31:0] a = get_field(r, off, L), xa = sext(a, L);
                        logic signed [32:0] k = is_signed ? $signed({xa[31], xa}) : $signed({1'b0, a});
                        if (!found || (is_min ? (k < best) : (k > best))) best = k;
                        found = 1'b1;
                    end
                    if (found) begin
                        for (int r = 0; r < ROWS; r++) if (m_tag[r]) begin
                            logic [31:0] a = get_field(r, off, L), xa = sext(a, L);
                            logic signed [32:0] k = is_signed ? $signed({xa[31], xa}) : $signed({1'b0, a});
                            m_tag[r] = (k == best);
                        end
                        value = best[31:0];
                    end
                end
            endcase
        end

        3'd4: begin                                         // element
            int off = f_off(rs1[15:0]), L = f_len(rs1[15:0]);
            int r = int'(rs1[31:16]);
            if (f7 == 7'd0) begin
                has_rd = 1'b1;
                value  = get_field(r, off, L);
            end else begin
                set_field(r, off, L, rs2 & lenmask(L));
            end
        end

        3'd5: begin                                         // transpose buffer
            int off = f_off(rs1[15:0]), L = f_len(rs1[15:0]);
            int seg = int'(rs1[31:21]);
            int n   = (rs2 == 0 || rs2 >= 32) ? 32 : int'(rs2);
            case (f7)
                7'd0: m_xbuf[rs1[4:0]] = rs2;
                7'd1: begin has_rd = 1'b1; value = m_xbuf[rs1[4:0]]; end
                7'd2: if (seg < CHUNKS)
                          for (int k = 0; k < n; k++)
                              set_field(seg * 32 + k, off, L, m_xbuf[k] & lenmask(L));
                default:
                    for (int k = 0; k < 32; k++)
                        m_xbuf[k] = (seg < CHUNKS && k < n) ? get_field(seg * 32 + k, off, L) : '0;
            endcase
        end

        3'd7: begin                                         // info
            has_rd = 1'b1;
            case (rs1)
                32'd0:   value = 32'(ROWS);
                32'd1:   value = 32'(COLS);
                32'd2:   value = 32'h0002_0000;
                default: value = '0;
            endcase
        end

        default: ;
        endcase
    endtask

    // --------------------------------------------------------------- driver
    int unsigned errors = 0;
    int unsigned latency_sum [8][16];
    int unsigned latency_cnt [8][16];

    task automatic check_state(string what);
        for (int r = 0; r < ROWS; r++)
            if (dut_rows[r] !== m_rows[r]) begin
                if (errors < 10)
                    $display("MISMATCH after %s: row %0d dut=%h model=%h", what, r, dut_rows[r], m_rows[r]);
                errors++;
            end
        if (dut.tag !== m_tag) begin
            if (errors < 10) $display("MISMATCH after %s: tag dut=%h model=%h", what, dut.tag, m_tag);
            errors++;
        end
        for (int k = 0; k < 32; k++)
            if (dut.xbuf[k] !== m_xbuf[k]) begin
                if (errors < 10)
                    $display("MISMATCH after %s: xbuf[%0d] dut=%h model=%h", what, k, dut.xbuf[k], m_xbuf[k]);
                errors++;
            end
    endtask

    task automatic run_cmd(logic [2:0] f3, logic [6:0] f7, logic [31:0] rs1, logic [31:0] rs2);
        logic        exp_wr;
        logic [31:0] exp_val;
        int          cycles = 0;
        string       what;

        what = $sformatf("cmd f3=%0d f7=%0d rs1=%h rs2=%h", f3, f7, rs1, rs2);

        @(negedge clk);
        while (!cmd_ready) @(negedge clk);
        cmd_funct3 = f3; cmd_funct7 = f7; cmd_rs1 = rs1; cmd_rs2 = rs2;
        #1;
        if (cmd_legal !== model_legal(f3, f7)) begin
            $display("MISMATCH: legal decode for %s", what);
            errors++;
        end
        cmd_valid = 1'b1;
        @(negedge clk);
        cmd_valid = 1'b0;
        cycles = 1;
        while (!rsp_valid) begin
            @(negedge clk);
            cycles++;
            if (cycles > 10000) $fatal(1, "timeout on %s", what);
        end

        model_exec(f3, f7, rs1, rs2, exp_wr, exp_val);
        if (rsp_wr !== exp_wr || (exp_wr && rsp_data !== exp_val)) begin
            if (errors < 10)
                $display("MISMATCH on %s: rsp wr=%b data=%h, expected wr=%b data=%h",
                         what, rsp_wr, rsp_data, exp_wr, exp_val);
            errors++;
        end
        if (model_legal(f3, f7) && f7 < 16) begin
            latency_sum[f3][f7[3:0]] += cycles;
            latency_cnt[f3][f7[3:0]]++;
        end
        @(negedge clk);
        check_state(what);
    endtask

    // ------------------------------------------------------ random stimulus
    function automatic int rnd(int n); return int'($urandom % n); endfunction

    // A field descriptor; mostly inside the array, sometimes partly outside.
    function automatic logic [15:0] rnd_field(int maxlen = 32);
        int len = 1 + rnd(maxlen);
        int off = (rnd(8) == 0) ? rnd(COLS + 8) : rnd((COLS > len) ? COLS - len + 1 : 1);
        return {5'(len - 1), 11'(off)};
    endfunction

    // Sources must not partially overlap the destination from above (a
    // documented restriction): retry until each source starts at or above d,
    // or does not overlap it.
    function automatic logic ok_overlap(logic [15:0] d, logic [15:0] s);
        int d0 = f_off(d), d1 = f_off(d) + f_len(d);
        int s0 = f_off(s), s1 = f_off(s) + f_len(s);
        return (s0 >= d0) || (s1 <= d0) || (s0 >= d1);
    endfunction

    function automatic logic [31:0] rnd_scalar();
        case (rnd(4))
            0:       return 32'(rnd(4));
            1:       return '1;
            default: return $urandom;
        endcase
    endfunction

    function automatic logic [31:0] rnd_rowarg();
        return (rnd(10) == 0) ? 32'(ROWS + rnd(64)) : 32'(rnd(ROWS));
    endfunction

    task automatic random_cmd();
        logic [2:0]  f3;
        logic [6:0]  f7;
        logic [31:0] rs1, rs2;
        logic [15:0] d, a, b;
        logic [31:0] rowarg = rnd_rowarg();

        case (rnd(20))
        0,1,2,3,4,5: begin                                   // arithmetic
            f3 = (rnd(2) == 0) ? 3'd0 : 3'd1;
            f7 = 7'(rnd(11));
            do begin
                d = rnd_field();
                a = (rnd(4) == 0) ? d : rnd_field();
                b = (rnd(4) == 0) ? d : rnd_field();
            end while (!ok_overlap(d, a) || (f3 == 3'd0 && !ok_overlap(d, b)));
            rs1 = {a, d};
            rs2 = (f3 == 3'd1) ? rnd_scalar() : {16'(rnd(65536)), b};
        end
        6,7,8,9: begin                                       // compare
            f3  = 3'd2;
            f7  = {2'(rnd(3)), 1'(rnd(2)), 4'(rnd(10))};
            a   = rnd_field();
            rs1 = {16'(rnd(65536)), a};
            rs2 = f7[4] ? rnd_scalar() : {16'd0, (rnd(2) == 0) ? {a[15:11], 11'(rnd(COLS))} : rnd_field()};
        end
        10,11,12: begin                                      // tag
            f3  = 3'd3;
            f7  = 7'(rnd(12));
            rs1 = (f7 == 7'd3) ? 32'(rnd(ROWS + 8)) : {16'd0, rnd_field()};
            rs2 = (f7 == 7'd3) ? 32'(rnd(ROWS + 8)) : $urandom;
        end
        13,14: begin                                         // element
            f3  = 3'd4;
            f7  = 7'(rnd(2));
            rs1 = {rowarg[15:0], rnd_field()};
            rs2 = $urandom;
        end
        15,16,17: begin                                      // transpose buffer
            f3  = 3'd5;
            f7  = 7'(rnd(4));
            rs1 = (f7 <= 7'd1) ? 32'(rnd(32)) : {rowarg[15:0], rnd_field()};
            rs2 = (f7 <= 7'd1 || rnd(2) == 0) ? $urandom : 32'(rnd(40));
        end
        18: begin                                            // info
            f3  = 3'd7;
            f7  = 7'd0;
            rs1 = 32'(rnd(4));
            rs2 = '0;
        end
        default: begin                                       // any encoding
            f3  = 3'(rnd(8));
            f7  = 7'(rnd(128));
            rs1 = $urandom;
            rs2 = $urandom;
        end
        endcase
        run_cmd(f3, f7, rs1, rs2);
    endtask

    // ---------------------------------------------------------- directed
    function automatic logic [15:0] fld(int off, int len); return {5'(len - 1), 11'(off)}; endfunction

    task automatic directed();
        // a = row index, b = 3 * row, d = a + b, then find rows with d == 8.
        for (int r = 0; r < ROWS; r++) begin
            run_cmd(3'd4, 7'd1, {16'(r), fld(0, 8)}, 32'(r));
            run_cmd(3'd4, 7'd1, {16'(r), fld(8, 8)}, 32'(3 * r));
        end
        run_cmd(3'd0, 7'd0, {fld(0, 8), fld(16, 8)}, {16'd0, fld(8, 8)});
        run_cmd(3'd2, 7'd0 | 7'h10, {16'd0, fld(16, 8)}, 32'd8);    // EQ scalar, SET
        run_cmd(3'd3, 7'd6, '0, '0);                                 // COUNT
        run_cmd(3'd3, 7'd7, '0, '0);                                 // FIRST
        run_cmd(3'd3, 7'd0, '0, '0);                                 // ALL
        run_cmd(3'd3, 7'd8, {16'd0, fld(16, 8)}, '0);                // MAXU
    endtask

    // ----------------------------------------------------------------- main
    initial begin
        int unsigned seed = 1, ncmds = 3000;
        if ($value$plusargs("seed=%d", seed)) ;
        if ($value$plusargs("ncmds=%d", ncmds)) ;
        void'($urandom(seed));

        for (int r = 0; r < ROWS; r++) m_rows[r] = '0;
        m_tag = '1;
        for (int k = 0; k < 32; k++) m_xbuf[k] = '0;

        repeat (3) @(negedge clk);
        rst_n = 1'b1;

        directed();
        for (int n = 0; n < int'(ncmds); n++) random_cmd();

        $display("tb_ap_core ROWS=%0d COLS=%0d seed=%0d: %0d commands, %0d errors",
                 ROWS, COLS, seed, ncmds, errors);
        if ($test$plusargs("latency")) begin
            $display("average latency in cycles (group, op, cycles):");
            for (int g = 0; g < 8; g++)
                for (int o = 0; o < 16; o++)
                    if (latency_cnt[g][o] != 0)
                        $display("  %0d %2d %0.1f", g, o, real'(latency_sum[g][o]) / latency_cnt[g][o]);
        end
        if (errors == 0) $display("PASS");
        else             $display("FAIL");
        $finish;
    end

endmodule
