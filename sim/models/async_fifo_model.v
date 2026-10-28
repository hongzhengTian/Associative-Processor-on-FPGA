`timescale 1ps/1ps
// Behavioral independent-clock FIFO, standing in for the Xilinx FIFO Generator
// configured as "Independent Clocks Block RAM, Standard FIFO" (dout is valid
// one rd_clk cycle after rd_en). The write pointer reaches the read domain, and
// the read pointer the write domain, through two-flop synchronizers, so the
// empty/full flags lag the other side by a few cycles as in the real IP.
module async_fifo_model
#(
    parameter WIDTH      = 8,
    parameter ADDR_BITS  = 8
)
(
    input                   rst,        // asynchronous, active high
    input                   wr_clk,
    input                   rd_clk,
    input  [WIDTH - 1 : 0]  din,
    input                   wr_en,
    input                   rd_en,
    output reg [WIDTH - 1 : 0] dout,
    output                  full,
    output                  empty,
    output                  wr_rst_busy,
    output                  rd_rst_busy
);

    localparam DEPTH = 1 << ADDR_BITS;

    reg [WIDTH - 1 : 0] mem [0 : DEPTH - 1];
    reg [ADDR_BITS : 0] wr_ptr, rd_ptr;
    reg [ADDR_BITS : 0] wr_ptr_rd_s1, wr_ptr_rd_s2;
    reg [ADDR_BITS : 0] rd_ptr_wr_s1, rd_ptr_wr_s2;

    assign empty       = (rd_ptr == wr_ptr_rd_s2);
    assign full        = (wr_ptr - rd_ptr_wr_s2) >= DEPTH - 1;
    assign wr_rst_busy = rst;
    assign rd_rst_busy = rst;

    always @(posedge wr_clk or posedge rst) begin
        if (rst) begin
            wr_ptr       <= 0;
            rd_ptr_wr_s1 <= 0;
            rd_ptr_wr_s2 <= 0;
        end
        else begin
            rd_ptr_wr_s1 <= rd_ptr;
            rd_ptr_wr_s2 <= rd_ptr_wr_s1;
            if (wr_en) begin
                if (full)
                    $display("%t %m: WARNING write while full, data dropped", $time);
                else begin
                    mem[wr_ptr[ADDR_BITS - 1 : 0]] <= din;
                    wr_ptr <= wr_ptr + 1;
                end
            end
        end
    end

    always @(posedge rd_clk or posedge rst) begin
        if (rst) begin
            rd_ptr       <= 0;
            wr_ptr_rd_s1 <= 0;
            wr_ptr_rd_s2 <= 0;
            dout         <= 0;
        end
        else begin
            wr_ptr_rd_s1 <= wr_ptr;
            wr_ptr_rd_s2 <= wr_ptr_rd_s1;
            if (rd_en && !empty) begin
                dout   <= mem[rd_ptr[ADDR_BITS - 1 : 0]];
                rd_ptr <= rd_ptr + 1;
            end
        end
    end

endmodule

// Port-compatible replacements for the two FIFO Generator instances in AP_top.
module fifo_generator_0 (
    input           rst,
    input           wr_clk,
    input           rd_clk,
    input  [38 : 0] din,
    input           wr_en,
    input           rd_en,
    output [38 : 0] dout,
    output          full,
    output          empty,
    output          wr_rst_busy,
    output          rd_rst_busy
);
    async_fifo_model #(.WIDTH(39), .ADDR_BITS(8)) u_fifo (
        .rst(rst), .wr_clk(wr_clk), .rd_clk(rd_clk), .din(din), .wr_en(wr_en),
        .rd_en(rd_en), .dout(dout), .full(full), .empty(empty),
        .wr_rst_busy(wr_rst_busy), .rd_rst_busy(rd_rst_busy));
endmodule

module fifo_generator_1 (
    input           rst,
    input           wr_clk,
    input           rd_clk,
    input  [52 : 0] din,
    input           wr_en,
    input           rd_en,
    output [52 : 0] dout,
    output          full,
    output          empty,
    output          wr_rst_busy,
    output          rd_rst_busy
);
    async_fifo_model #(.WIDTH(53), .ADDR_BITS(7)) u_fifo (
        .rst(rst), .wr_clk(wr_clk), .rd_clk(rd_clk), .din(din), .wr_en(wr_en),
        .rd_en(rd_en), .dout(dout), .full(full), .empty(empty),
        .wr_rst_busy(wr_rst_busy), .rd_rst_busy(rd_rst_busy));
endmodule
