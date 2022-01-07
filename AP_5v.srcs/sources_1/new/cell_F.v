module cell_F
#(parameter DATA_DEPTH = 4)
  (     
  input wire [DATA_DEPTH - 1 : 0] Ip,
  input wire rstIn,
  input wire Key,
  input wire Mask,
  input wire [2:0]Pass,
  input wire [DATA_DEPTH - 1 : 0]tag,
  input wire clk,
  input wire ABS_opt,
  input wire [DATA_DEPTH - 1 : 0]Q_S,
  output reg [DATA_DEPTH - 1 : 0]Q,
  output reg [DATA_DEPTH - 1 : 0]tag_cell
  );

  reg [DATA_DEPTH - 1 : 0]Qb;
  reg [DATA_DEPTH - 1 : 0]D;
  reg [DATA_DEPTH - 1 : 0]Ie;
  
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
    
  always@(tag, Ie, Pass, Ip, Q, Qb, ABS_opt,Q_S)
  begin
  for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) 
    begin
        if (Ie[i] == 1)
            begin
                D[i] = Ip[i];
            end

        /*else if (ABS_opt == 0) 
            begin
                if ((Ie[i] == 0)&&(tag[i] == 1)&&(Pass == 1))
                    begin
                        D[i] <= Q[i];
                    end
                else if ((Ie[i] == 0)&&(tag[i] == 1)&&(Pass == 2))
                    begin
                        D[i] <= Q[i];
                    end
                else if ((Ie[i] == 0)&&(tag[i] == 1)&&(Pass == 3))
                    begin
                        D[i] <= Qb[i];
                    end
                else D[i] <= Q[i];
            end*/
        else if ((Ie[i] == 0)&&(tag[i] == 1)&&(Pass == 3)&&(ABS_opt == 0)) D[i] = Qb[i];
        else if ((Ie[i] == 0)&&(tag[i] == 1)&&(Q_S[i] == 1)&&(Pass == 4)) D[i] = Qb[i];
        /*else if (ABS_opt == 1)
            begin
                if ((Ie[i] == 0)&&(tag[i] == 1)&&(Q_S[i] == 0)&&(Pass == 1))
                    begin
                        D[i] <= Q[i];
                    end
                else if ((Ie[i] == 0)&&(tag[i] == 1)&&(Q_S[i] == 1)&&(Pass == 2))
                    begin
                        D[i] <= Q[i];
                    end
                else if ((Ie[i] == 0)&&(tag[i] == 1)&&(Q_S[i] == 1)&&(Pass == 3))
                    begin
                        D[i] <= Q[i];
                    end
                else if ((Ie[i] == 0)&&(tag[i] == 1)&&(Q_S[i] == 1)&&(Pass == 4))
                    begin
                        D[i] <= Qb[i];
                    end
                else D[i] <= Q[i];
            end
        else if ((Ie[i] == 0)&&(tag[i] == 0))
                    begin
                        D[i] <= Q[i];
                    end*/
        else D[i] = Q[i];
    end
    end
    
    always @(posedge clk)
    for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
        begin
            Q[i] <= D[i];
            Qb[i] <= ~D[i];
        end
    end
        
   always @(Mask or Q or Qb or Key or clk)
   for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
    begin
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
