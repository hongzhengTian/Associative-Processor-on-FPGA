module cell_C
#(
parameter DATA_DEPTH = 128
)
(
input wire [DATA_DEPTH - 1 : 0] input_C,
input wire                      key,
input wire                      mask,
input wire [2:0]                pass,
input wire [DATA_DEPTH - 1 : 0] tag,
input wire                      rst_In,
input wire                      clk,
output reg [DATA_DEPTH - 1 : 0] Q,
output reg [DATA_DEPTH - 1 : 0] tag_cell
);
  
reg [DATA_DEPTH - 1 : 0] Ie;
reg [DATA_DEPTH - 1 : 0] Qb;
reg [DATA_DEPTH - 1 : 0] D;
  
integer i;
  
always @(rst_In) begin
    for (i = 0; i < DATA_DEPTH; i = i + 1) begin
        if (!rst_In) begin
            Ie[i] = 1'b1;
        end
        else begin
            Ie[i] = 1'b0;
        end
    end
end

always@(tag, Ie, pass, input_C, Qb, Q) begin
    for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
        if (Ie[i] == 1) begin
            D[i] = input_C[i];
        end
        else if ((Ie[i] == 0) && (tag[i] == 1)) begin
            case (pass)
                1: D[i] = Qb[i];
                2: D[i] = Q[i];
                3: D[i] = Qb[i];
                4: D[i] = Q[i];
                default: D[i] = Q[i];
            endcase
        end
        else begin
            D[i] = Q[i];
        end
    end
end
    
always @(posedge clk) begin
    for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
        Q[i] <= D[i];
        Qb[i] <= ~D[i];
    end
end

always @(mask or key or clk or Qb or Q) begin
   for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
        case ({mask, key})
            2'b00: tag_cell[i] = 1'b1;
            2'b01: tag_cell[i] = 1'b1;
            2'b10: tag_cell[i] = Qb[i];
            2'b11: tag_cell[i] = Q[i];
            default: ;
        endcase
    end
end
    
endmodule
