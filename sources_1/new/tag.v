module tag_cell
#(
parameter DATA_DEPTH = 128
)
(
input wire [DATA_DEPTH - 1 : 0] tag_A,
input wire [DATA_DEPTH - 1 : 0] tag_B,
input wire [DATA_DEPTH - 1 : 0] tag_C,
input wire [DATA_DEPTH - 1 : 0] tag_F_TSC,
input wire                      clk,
input wire                      rst,
output reg [DATA_DEPTH - 1 : 0] tag,
output reg [DATA_DEPTH - 1 : 0] tag_TSC
);

reg [DATA_DEPTH - 1 : 0] tag_and;
reg [DATA_DEPTH - 1 : 0] tag_and_TSC;

always@(tag_A or tag_B or tag_C or tag_F_TSC) begin
    tag_and = tag_A & tag_B & tag_C;
    tag_and_TSC = tag_A & tag_F_TSC;
end

always @(posedge clk or negedge rst) begin
    if (!rst) begin
        tag <= 0;
        tag_TSC <= 0;
    end
    else begin
        tag <= tag_and;
        tag_TSC <= tag_and_TSC;
    end
end
    
endmodule