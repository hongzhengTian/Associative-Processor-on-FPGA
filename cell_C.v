module cell_C
#(parameter DATA_DEPTH = 128)(
  input wire [DATA_DEPTH - 1 : 0] Ip,
  input wire Key,
  input wire Mask,
  input wire [2:0]Pass,
  input wire [DATA_DEPTH - 1 : 0]tag,
  input wire rstIn,
  input wire clk,
  output reg [DATA_DEPTH - 1 : 0]Q,
  output reg [DATA_DEPTH - 1 : 0]tag_cell
  );
  
  reg [DATA_DEPTH - 1 : 0]Ie;
  reg [DATA_DEPTH - 1 : 0]Qb;
  reg [DATA_DEPTH - 1 : 0]D;
  
  integer i;
  
  always @(rstIn)
    begin
        for (i = 0; i < DATA_DEPTH; i = i + 1)begin
            if(!rstIn)begin
            Ie[i] = 1'b1;
            end
            else Ie[i] = 1'b0;
        end
    end
  
  always@(tag, Ie, Pass, Ip, Qb, Q)
  for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
    begin
        if (Ie[i] == 1) D[i] <= Ip[i];
        /*else if ((Ie[i] == 0)&&(tag[i] == 1)&&(Pass == 1))  D[i] <= Qb[i];
        else if ((Ie[i] == 0)&&(tag[i] == 1)&&(Pass == 2))  D[i] <= Q[i];
        else if ((Ie[i] == 0)&&(tag[i] == 1)&&(Pass == 3))  D[i] <= Qb[i];
        else if ((Ie[i] == 0)&&(tag[i] == 1)&&(Pass == 4))  D[i] <= Q[i];*/

        else if ((Ie[i] == 0)&&(tag[i] == 1))
        begin
            case (Pass)
            1: D[i] <= Qb[i];
            2: D[i] <= Q[i];
            3: D[i] <= Qb[i];
            4: D[i] <= Q[i];
            default: D[i] <= Q[i];
            endcase
        end
        

        else D[i] <= Q[i];
    end
    end
    
    always @(posedge clk)
    for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
        begin
            Q[i] <= D[i];
            Qb[i] <= ~D[i];
        end
    end
        
   always @(Mask or Key or clk or Qb or Q)
   for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
    begin
        case ({Mask,Key})
            2'b00: tag_cell[i] <= 1'b1;
            2'b01: tag_cell[i] <= 1'b1;
            2'b10: tag_cell[i] <= Qb[i];
            2'b11: tag_cell[i] <= Q[i];
            default: ;
        endcase
    end
    end
    
endmodule
