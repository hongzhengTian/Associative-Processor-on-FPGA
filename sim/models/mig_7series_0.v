`timescale 1ps/1ps
// Behavioral model of the mig_7series_0 user (app_*) interface, for simulating
// AP_top with open-source simulators instead of the encrypted Xilinx MIG IP.
// The DDR3 pins are left undriven; the memory lives inside this model, one
// 128-bit word per command (BL8 on the x16 DDR3), so app_addr advances by 8.
//
// The defaults are calibrated against the original Vivado 2018.3 xsim run
// (MIG + Micron ddr3_model): with them the testbench prints exactly the same
// 128 results, and the DDR traffic lines up with that run to within 1%.
// ddr_controller / DDR_cache_interface depend on details of MIG timing, so
// the model has to reproduce these on purpose:
//
//   - ui_clk has the same frequency and phase as sys_clk_i (200 MHz in, 4:1).
//     Its edges are released in the inactive region (#0): after every process
//     woken by sys_clk_i, before any non-blocking update. That is the order the
//     PLL/BUFG models give in xsim. The ui_clk logic thus sees the old values of
//     sys_clk_i_2 registers but the new values of the testbench's blocking
//     writes, which is how one padding word (not two) ends up at the start of
//     each write phase, as data_cache and the "+1" burst lengths expect.
//   - app_rdy drops for BURST_STALL_CYCLES cycles, BURST_STALL_AT cycles into
//     every burst (app_en held high), like MIG does while it opens a row. In a
//     store burst ddr_controller presents each address for two cycles and ends
//     one command short of its data beats; this stall (an even number of cycles)
//     supplies the missing command. Without it the last word of every store is
//     wrong and the data beat spills into the next burst.
//   - Commands retire in order, one per ui_clk cycle, plus ROW_SWITCH_CYCLES
//     when the DDR row (app_addr[27:10]) changes. A write waits for its data
//     beat. Read data returns RD_LATENCY cycles after the command retires.
//   - app_rdy / app_wdf_rdy also drop while CMD_QUEUE_DEPTH commands or
//     WDF_QUEUE_DEPTH data beats are waiting.
//   - init_calib_complete rises at CALIB_DONE_PS (absolute time), as in the
//     xsim run, so the interrupt that AP_top_tb raises at a fixed time arrives
//     at the same point of the program.
//
// The design is sensitive to these settings (other values can hang it or give
// wrong results), so change them only to explore that. Each can be overridden
// at run time: +mig_calib_done_ps= +mig_rd_latency= +mig_row_switch=
// +mig_cmd_depth= +mig_wdf_depth= +mig_stall_at= +mig_stall_len=
module mig_7series_0
#(
    parameter CALIB_DONE_PS     = 54000000,
    parameter RD_LATENCY        = 20,       // at most MAX_RD_LATENCY
    parameter CMD_QUEUE_DEPTH   = 16,       // app_rdy drops when this many commands wait
    parameter WDF_QUEUE_DEPTH   = 16,       // app_wdf_rdy drops when this many beats wait
    parameter ROW_SWITCH_CYCLES = 12,       // extra ui_clk cycles to open a new row
    parameter BURST_STALL_AT    = 1,        // app_rdy low BURST_STALL_CYCLES cycles,
    parameter BURST_STALL_CYCLES = 2,       // this many cycles into each burst
    parameter MEM_ADDR_BITS     = 16        // words of 128 bit
)
(
    output [13:0]   ddr3_addr,
    output [2:0]    ddr3_ba,
    output          ddr3_cas_n,
    output [0:0]    ddr3_ck_n,
    output [0:0]    ddr3_ck_p,
    output [0:0]    ddr3_cke,
    output          ddr3_ras_n,
    output          ddr3_reset_n,
    output          ddr3_we_n,
    inout  [15:0]   ddr3_dq,
    inout  [1:0]    ddr3_dqs_n,
    inout  [1:0]    ddr3_dqs_p,
    output          init_calib_complete,
    output [0:0]    ddr3_cs_n,
    output [1:0]    ddr3_dm,
    output [0:0]    ddr3_odt,

    input  [27:0]   app_addr,
    input  [2:0]    app_cmd,
    input           app_en,
    input  [127:0]  app_wdf_data,
    input           app_wdf_end,
    input  [15:0]   app_wdf_mask,
    input           app_wdf_wren,
    output [127:0]  app_rd_data,
    output          app_rd_data_end,
    output          app_rd_data_valid,
    output          app_rdy,
    output          app_wdf_rdy,
    input           app_sr_req,
    input           app_ref_req,
    input           app_zq_req,
    output          app_sr_active,
    output          app_ref_ack,
    output          app_zq_ack,
    output          ui_clk,
    output          ui_clk_sync_rst,
    output [11:0]   device_temp,

    input           sys_clk_i,
    input           sys_rst                 // active low (RST_ACT_LOW = 1)
);

    localparam CMD_WRITE = 3'b000;
    localparam CMD_READ  = 3'b001;
    localparam QUEUE_DEPTH = 64;            // storage only, must exceed both depths above
    localparam MAX_RD_LATENCY = 64;

    assign ddr3_addr     = 14'd0;
    assign ddr3_ba       = 3'd0;
    assign ddr3_cas_n    = 1'b1;
    assign ddr3_ck_n     = 1'b1;
    assign ddr3_ck_p     = 1'b0;
    assign ddr3_cke      = 1'b0;
    assign ddr3_ras_n    = 1'b1;
    assign ddr3_reset_n  = sys_rst;
    assign ddr3_we_n     = 1'b1;
    assign ddr3_cs_n     = 1'b1;
    assign ddr3_dm       = 2'b00;
    assign ddr3_odt      = 1'b0;
    assign ddr3_dq       = 16'bz;
    assign ddr3_dqs_n    = 2'bz;
    assign ddr3_dqs_p    = 2'bz;
    assign app_sr_active = 1'b0;
    assign app_ref_ack   = 1'b0;
    assign app_zq_ack    = 1'b0;
    assign device_temp   = 12'd0;

    /* clock and reset */
    reg ui_clk_r = 1'b0;
    always @(sys_clk_i) #0 ui_clk_r = sys_clk_i;
    assign ui_clk = ui_clk_r;

    reg [3:0] rst_sync = 4'hf;
    always @(posedge ui_clk or negedge sys_rst) begin
        if (!sys_rst)
            rst_sync <= 4'hf;
        else
            rst_sync <= {rst_sync[2:0], 1'b0};
    end
    assign ui_clk_sync_rst = rst_sync[3];

    integer calib_done_ps   = CALIB_DONE_PS;
    integer cmd_queue_depth = CMD_QUEUE_DEPTH;
    integer wdf_queue_depth = WDF_QUEUE_DEPTH;
    integer row_switch      = ROW_SWITCH_CYCLES;
    integer rd_latency      = RD_LATENCY;
    integer stall_at        = BURST_STALL_AT;
    integer stall_len       = BURST_STALL_CYCLES;
    initial begin
        if ($value$plusargs("mig_calib_done_ps=%d", calib_done_ps)) ;
        if ($value$plusargs("mig_cmd_depth=%d", cmd_queue_depth)) ;
        if ($value$plusargs("mig_wdf_depth=%d", wdf_queue_depth)) ;
        if ($value$plusargs("mig_row_switch=%d", row_switch)) ;
        if ($value$plusargs("mig_rd_latency=%d", rd_latency)) ;
        if ($value$plusargs("mig_stall_at=%d", stall_at)) ;
        if ($value$plusargs("mig_stall_len=%d", stall_len)) ;
        if (rd_latency < 2 || rd_latency > MAX_RD_LATENCY)
            $display("mig_7series_0 model: ERROR read latency must be 2..%0d", MAX_RD_LATENCY);
        if (cmd_queue_depth >= QUEUE_DEPTH || wdf_queue_depth >= QUEUE_DEPTH)
            $display("mig_7series_0 model: ERROR queue depth must be below %0d", QUEUE_DEPTH);
    end

    reg calib_done = 1'b0;
    initial begin
        #1;
        #(calib_done_ps - 1);
        @(posedge ui_clk);
        calib_done <= 1'b1;
    end
    assign init_calib_complete = calib_done & ~ui_clk_sync_rst;

    /* ready flags */
    reg     cmd_space = 1'b1;
    reg     wdf_space = 1'b1;
    integer burst_cyc = 0;      // cycles app_en has been high in a row
    always @(posedge ui_clk) burst_cyc <= (app_en && !ui_clk_sync_rst) ? burst_cyc + 1 : 0;
    wire burst_stall = app_en && burst_cyc >= stall_at && burst_cyc < stall_at + stall_len;
    assign app_rdy             = init_calib_complete & cmd_space & ~burst_stall;
    assign app_wdf_rdy         = init_calib_complete & wdf_space;

    /* memory, all zero like an unwritten ddr3_model */
    reg [127:0] mem [0 : (1 << MEM_ADDR_BITS) - 1];
    integer i, j;
    initial for (i = 0; i < (1 << MEM_ADDR_BITS); i = i + 1) mem[i] = 128'd0;

    function [MEM_ADDR_BITS - 1 : 0] word_index;
        input [27:0] addr;
        begin
            if (addr[2:0] != 3'd0 || (addr >> (MEM_ADDR_BITS + 3)) != 0)
                $display("%t mig_7series_0 model: WARNING address 0x%h is unaligned or outside the modelled range", $time, addr);
            word_index = addr[MEM_ADDR_BITS + 2 : 3];
        end
    endfunction

    /* command and write data queues */
    reg [2:0]   cmd_q_cmd  [0 : QUEUE_DEPTH - 1];
    reg [27:0]  cmd_q_addr [0 : QUEUE_DEPTH - 1];
    reg [127:0] wdf_q      [0 : QUEUE_DEPTH - 1];
    integer cmd_wr = 0, cmd_rd = 0, wdf_wr = 0, wdf_rd = 0;

    /* read return pipeline */
    reg [127:0] rd_pipe_data  [0 : MAX_RD_LATENCY - 1];
    reg         rd_pipe_valid [0 : MAX_RD_LATENCY - 1];
    initial for (j = 0; j < MAX_RD_LATENCY; j = j + 1) rd_pipe_valid[j] = 1'b0;

    reg [127:0] issue_data;
    reg         issue_valid;
    reg [17:0]  open_row = 18'h3ffff;
    integer     busy = 0;

    always @(posedge ui_clk) begin
        if (ui_clk_sync_rst) begin
            cmd_wr = 0; cmd_rd = 0; wdf_wr = 0; wdf_rd = 0;
            busy = 0;
            cmd_space <= 1'b1;
            wdf_space <= 1'b1;
            for (i = 0; i < MAX_RD_LATENCY; i = i + 1) rd_pipe_valid[i] <= 1'b0;
        end
        else begin
            if (app_en && app_rdy) begin
                if (cmd_wr - cmd_rd >= QUEUE_DEPTH)
                    $display("%t mig_7series_0 model: ERROR command queue overflow", $time);
                cmd_q_cmd [cmd_wr % QUEUE_DEPTH] = app_cmd;
                cmd_q_addr[cmd_wr % QUEUE_DEPTH] = app_addr;
                cmd_wr = cmd_wr + 1;
            end
            if (app_wdf_wren && app_wdf_rdy) begin
                if (wdf_wr - wdf_rd >= QUEUE_DEPTH)
                    $display("%t mig_7series_0 model: ERROR write data queue overflow", $time);
                wdf_q[wdf_wr % QUEUE_DEPTH] = app_wdf_data;
                wdf_wr = wdf_wr + 1;
            end

            /* retire at most one command per cycle, in order */
            issue_valid = 1'b0;
            issue_data  = 128'd0;
            if (busy > 0)
                busy = busy - 1;
            else if (row_switch > 0 && cmd_rd != cmd_wr && cmd_q_addr[cmd_rd % QUEUE_DEPTH][27:10] != open_row) begin
                open_row = cmd_q_addr[cmd_rd % QUEUE_DEPTH][27:10];
                busy = row_switch - 1;
            end
            else if (cmd_rd != cmd_wr) begin
                case (cmd_q_cmd[cmd_rd % QUEUE_DEPTH])
                    CMD_WRITE: if (wdf_rd != wdf_wr) begin
                        mem[word_index(cmd_q_addr[cmd_rd % QUEUE_DEPTH])] = wdf_q[wdf_rd % QUEUE_DEPTH];
                        wdf_rd = wdf_rd + 1;
                        cmd_rd = cmd_rd + 1;
                    end
                    CMD_READ: begin
                        issue_valid = 1'b1;
                        issue_data  = mem[word_index(cmd_q_addr[cmd_rd % QUEUE_DEPTH])];
                        cmd_rd = cmd_rd + 1;
                    end
                    default: begin
                        $display("%t mig_7series_0 model: WARNING unsupported app_cmd %b ignored", $time, cmd_q_cmd[cmd_rd % QUEUE_DEPTH]);
                        cmd_rd = cmd_rd + 1;
                    end
                endcase
            end

            cmd_space <= (cmd_wr - cmd_rd) < cmd_queue_depth;
            wdf_space <= (wdf_wr - wdf_rd) < wdf_queue_depth;

            rd_pipe_valid[0] <= issue_valid;
            rd_pipe_data[0]  <= issue_data;
            for (i = 1; i < MAX_RD_LATENCY; i = i + 1) begin
                rd_pipe_valid[i] <= rd_pipe_valid[i - 1];
                rd_pipe_data[i]  <= rd_pipe_data[i - 1];
            end
        end
    end

    /* like the real MIG, app_rd_data holds the last beat between valids */
    reg [127:0] rd_data_hold = 128'd0;
    always @(posedge ui_clk)
        if (rd_pipe_valid[rd_latency - 2])
            rd_data_hold <= rd_pipe_data[rd_latency - 2];
    assign app_rd_data       = rd_data_hold;
    assign app_rd_data_valid = rd_pipe_valid[rd_latency - 1];
    assign app_rd_data_end   = rd_pipe_valid[rd_latency - 1];

endmodule
