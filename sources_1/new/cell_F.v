module cell_F
#(
parameter DATA_DEPTH = 4
)
( 
input wire [DATA_DEPTH - 1 : 0] input_F,
input wire                      rst_In,
input wire                      key,
input wire                      mask,
input wire [2:0]                pass,
input wire [DATA_DEPTH - 1 : 0] tag,
input wire                      clk,
input wire                      abs_opt,
input wire [DATA_DEPTH - 1 : 0] Q_S,
output reg [DATA_DEPTH - 1 : 0] Q,
output reg [DATA_DEPTH - 1 : 0] tag_cell
);

reg [DATA_DEPTH - 1 : 0] Qb;
reg [DATA_DEPTH - 1 : 0] D;
reg [DATA_DEPTH - 1 : 0] Ie;

integer i;

always @(rst_In) begin
    for (i = 0; i < DATA_DEPTH; i = i + 1) begin
        if(!rst_In) begin
            Ie[i] = 1'b1;
        end
        else begin
            Ie[i] = 1'b0;
        end
    end
end

always@(tag, Ie, pass, input_F, Q, Qb, abs_opt,Q_S) begin
    for (i = 0; i <= DATA_DEPTH - 1; i = i + 1)  begin
        if (Ie[i] == 1) begin
            D[i] = input_F[i];
        end
        else if ((Ie[i] == 0) && (tag[i] == 1) && (pass == 3) && (abs_opt == 0)) begin
            D[i] = Qb[i];
        end
        else if ((Ie[i] == 0) && (tag[i] == 1) && (Q_S[i] == 1) && (pass == 4)) begin
            D[i] = Qb[i];
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
        
always @(mask or Q or Qb or key or clk) begin
    for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
        case ({mask,key})
            2'b00: tag_cell[i] = 1'b1;
            2'b01: tag_cell[i] = 1'b1;
            2'b10: tag_cell[i] = Qb[i];
            2'b11: tag_cell[i] = Q[i];
            default: ;
        endcase
    end
end
  
endmodule
