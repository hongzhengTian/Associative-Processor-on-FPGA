module cell_F
#(
parameter DATA_DEPTH = 4
)
( 
input wire [DATA_DEPTH - 1 : 0] Ip,
input wire                      rstIn,
input wire                      Key,
input wire                      Mask,
input wire [2:0]                Pass,
input wire [DATA_DEPTH - 1 : 0] tag,
input wire                      clk,
input wire                      ABS_opt,
input wire [DATA_DEPTH - 1 : 0] Q_S,
output reg [DATA_DEPTH - 1 : 0] Q,
output reg [DATA_DEPTH - 1 : 0] tag_cell
);

reg [DATA_DEPTH - 1 : 0] Qb;
reg [DATA_DEPTH - 1 : 0] D;
reg [DATA_DEPTH - 1 : 0] Ie;

integer i;

always @(rstIn) begin
    for (i = 0; i < DATA_DEPTH; i = i + 1) begin
        if(!rstIn) begin
            Ie[i] = 1'b1;
        end
        else begin
            Ie[i] = 1'b0;
        end
    end
end

always@(tag, Ie, Pass, Ip, Q, Qb, ABS_opt,Q_S) begin
    for (i = 0; i <= DATA_DEPTH - 1; i = i + 1)  begin
        if (Ie[i] == 1) begin
            D[i] = Ip[i];
        end
        else if ((Ie[i] == 0) && (tag[i] == 1) && (Pass == 3) && (ABS_opt == 0)) begin
            D[i] = Qb[i];
        end
        else if ((Ie[i] == 0) && (tag[i] == 1) && (Q_S[i] == 1) && (Pass == 4)) begin
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
        
always @(Mask or Q or Qb or Key or clk) begin
    for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
        case ({Mask,Key})
            2'b00: tag_cell[i] = 1'b1;
            2'b01: tag_cell[i] = 1'b1;
            2'b10: tag_cell[i] = Qb[i];
            2'b11: tag_cell[i] = Q[i];
            default: ;
        endcase
    end
end
  
endmodule
