module cell_B
  #(
  parameter DATA_WIDTH = 8,
  parameter DATA_DEPTH = 16,
  parameter ADDR_WIDTH_CAM = 8,
  parameter RowxRow = 3'd1,
  parameter ColxCol = 3'd2,
  parameter COPY_B = 3'd3,
  parameter COPY_R = 3'd4,
  parameter COPY_A = 3'd5
  )
  (
  input wire [DATA_WIDTH - 1 : 0]               Ip_row,
  input wire [DATA_DEPTH - 1 : 0]               Ip_col,
  input wire [DATA_WIDTH * DATA_DEPTH - 1 : 0]  Q_R,
  input wire [DATA_WIDTH * DATA_DEPTH - 1 : 0]  Q_A,
  input wire [ADDR_WIDTH_CAM - 1 : 0]           addr_input_Row,
  input wire [ADDR_WIDTH_CAM - 1 : 0]           addr_input_Col,
  input wire [2 : 0]                            input_mode,
  input wire                                    rstIn,
  input wire                                    Key,
  input wire [DATA_WIDTH - 1 : 0]               Mask,
  input wire                                    clk,
  input wire [DATA_DEPTH - 1 : 0]               tag,
  input wire [ADDR_WIDTH_CAM - 1 : 0]           addr_output_Row,
  input wire [ADDR_WIDTH_CAM - 1 : 0]           addr_output_Col,
  
  output reg [DATA_WIDTH - 1 : 0]               Q_out_row,
  output reg [DATA_DEPTH - 1 : 0]               Q_out_col,
  output reg [DATA_DEPTH - 1 : 0]               tag_row,
  output reg [DATA_WIDTH * DATA_DEPTH - 1 : 0]  Q
  );

  reg [DATA_DEPTH - 1 : 0]OutE_R;
  reg [DATA_WIDTH - 1 : 0]OutE_C;
  reg [DATA_DEPTH - 1 : 0] Ie_R;
  reg [DATA_WIDTH - 1 : 0] Ie_C;
  reg [DATA_WIDTH * DATA_DEPTH - 1 : 0]tag_cell;
  reg [DATA_WIDTH * DATA_DEPTH - 1 : 0]Qb; 
  reg [DATA_WIDTH - 1: 0] D [0 : DATA_DEPTH - 1]; 
  
  integer i, j;
  
    /*always @(addr_input or rstIn)
    begin
        for (i = 0; i < DATA_DEPTH; i = i + 1)begin
            if(!rstIn)begin
            Ie[i] = (addr_input==i)?1'b1:1'b0;
            end
            else Ie[i] = 1'b0;
        end
    end
  
    always@(tag, Mask,Ie,Ip,Q,Qb)
    for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
        for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
            begin
            case({tag[i],Mask[j],Ie[i]})
            //3'bX01: D[i][j] <= Ip[j];
            //3'bX11: D[i][j] <= Ip[j];
            3'b001: D[i][j] <= Ip[j];
            3'b011: D[i][j] <= Ip[j];
            3'b101: D[i][j] <= Ip[j];
            3'b111: D[i][j] <= Ip[j];
            3'b110: D[i][j] <= Qb[i*DATA_WIDTH + j];
            //3'b100: D[i][j] <= Q[i*DATA_WIDTH + j];
            //3'b000: D[i][j] <= Q[i*DATA_WIDTH + j];
            //3'b010: D[i][j] <= Q[i*DATA_WIDTH + j];
            
            default:D[i][j] <= Q[i*DATA_WIDTH + j];
            endcase
            end
        end
    end*/
    
    always @(*)
    begin
      if (input_mode == RowxRow)
      begin
          for (j = 0; j < DATA_WIDTH; j = j + 1) Ie_C[j] = 1'b1;
            for (i = 0; i < DATA_DEPTH; i = i + 1)
                begin
                    if(!rstIn) Ie_R[i] = (addr_input_Row==i)?1'b1:1'b0;
                    else Ie_R[i] = 1'b0;
                end
          for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
            for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
                begin
                    case({tag[i],Mask[j],(Ie_R[i]&Ie_C[j])})
                    //3'bX01: D[i][j] <= Ip[j];
                    //3'bX11: D[i][j] <= Ip[j];
                    3'b001: D[i][j] = Ip_row[j];
                    3'b011: D[i][j] = Ip_row[j];
                    3'b101: D[i][j] = Ip_row[j];
                    3'b111: D[i][j] = Ip_row[j];
                    3'b110: D[i][j] = Qb[i*DATA_WIDTH + j];
                    //3'b100: D[i][j] = Q[i*DATA_WIDTH + j];
                    //3'b000: D[i][j] = Q[i*DATA_WIDTH + j];
                    //3'b010: D[i][j] = Q[i*DATA_WIDTH + j];
                    default:D[i][j] = Q[i*DATA_WIDTH + j];
                    endcase
                end
            end
        end
      end

      else if (input_mode == ColxCol)
      begin
          for (j = 0; j < DATA_DEPTH; j = j + 1) Ie_R[j] = 1'b1;
            for (i = 0; i < DATA_WIDTH; i = i + 1)
                begin
                    if(!rstIn) Ie_C[i] = (addr_input_Col==i)?1'b1:1'b0;
                    else Ie_C[i] = 1'b0;
                end

          for (i = 0; i <= DATA_WIDTH - 1; i = i + 1) begin
            for (j = 0; j <= DATA_DEPTH - 1; j = j + 1) begin
                begin
                    case({tag[j],Mask[i],(Ie_R[j]&Ie_C[i])})
                    //3'bX01: D[i][j] <= Ip[j];
                    //3'bX11: D[i][j] = Ip[j];
                    3'b001: D[j][i] = Ip_col[j];
                    3'b011: D[j][i] = Ip_col[j];
                    3'b101: D[j][i] = Ip_col[j];
                    3'b111: D[j][i] = Ip_col[j];
                    3'b110: D[j][i] = Qb[j*DATA_WIDTH + i];
                    //3'b100: D[i][j] = Q[i*DATA_WIDTH + j];
                    //3'b000: D[i][j] = Q[i*DATA_WIDTH + j];
                    //3'b010: D[i][j] = Q[i*DATA_WIDTH + j];
                    default:D[j][i] = Q[j*DATA_WIDTH + i];
                    endcase
                end
            end
        end
      end

      else if (input_mode == COPY_A)
      begin
          for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
            for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
                 if(!rstIn) D[i][j] = Q_A[i*DATA_WIDTH + j];
                 else if (tag[i] == 1 && Mask[j] == 1 )
                        D[i][j] = Qb[i*DATA_WIDTH + j];
                 else D[i][j] = Q[i*DATA_WIDTH + j];
            end
        end
      end

      else if (input_mode == COPY_R)
      begin
          for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
            for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
                 if(!rstIn) D[i][j] = Q_R[i*DATA_WIDTH + j];
                 else if (tag[i] == 1 && Mask[j] == 1 )
                        D[i][j] = Qb[i*DATA_WIDTH + j];
                 else D[i][j] = Q[i*DATA_WIDTH + j];
            end
        end
      end

      else
      begin
          Ie_C = {{DATA_WIDTH}{1'b0}};
          Ie_R = {{DATA_DEPTH}{1'b0}};
          for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
            for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
                if (tag[i] == 1 && Mask[j] == 1 )
                    begin
                        D[i][j] = Qb[i*DATA_WIDTH + j];
                    end
                else D[i][j] = Q[i*DATA_WIDTH + j];
            end
        end
      end

      //else D[j][i] = Q[j*DATA_WIDTH + i];

    end

    always@(posedge clk)
    for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
        for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
            begin
                    Q[i*DATA_WIDTH+j] <= D[i][j];
                    Qb[i*DATA_WIDTH+j] <= ~D[i][j];
            end
        end
    end

    always @(addr_output_Col or addr_output_Row or input_mode or Q)
    begin
        if (input_mode == RowxRow)
        begin
            for (j = 0; j < DATA_WIDTH; j = j + 1) OutE_C[j] = 1'b1;
            for (i = 0; i < DATA_DEPTH; i = i + 1)begin
                OutE_R[i] = (addr_output_Row==i)?1'b1:1'b0;
            end
            for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
                    if((OutE_R[i]&OutE_C[j] == 1)) Q_out_row[j] <= Q[i*DATA_WIDTH + j];
                end
            end
        end

        else if(input_mode == ColxCol)
        begin
            for (j = 0; j < DATA_DEPTH; j = j + 1) OutE_R[j] = 1'b1;
            for (i = 0; i < DATA_WIDTH; i = i + 1)begin
                OutE_C[i] = (addr_output_Col==i)?1'b1:1'b0;
            end
            for (i = 0; i <= DATA_WIDTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_DEPTH - 1; j = j + 1) begin
                    if((OutE_R[j]&OutE_C[i] == 1)) Q_out_col[j] <= Q[j*DATA_WIDTH + i];
                end
            end
        end
    end
    
    always @(Mask or Key or clk or Q or Qb)
        for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
            for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
                begin
                    case ({Mask[j],Key})
                    2'b00: tag_cell[i*DATA_WIDTH + j] <= 1'b1;
                    2'b01: tag_cell[i*DATA_WIDTH + j] <= 1'b1;
                    2'b10: tag_cell[i*DATA_WIDTH + j] <= Qb[i*DATA_WIDTH + j];
                    2'b11: tag_cell[i*DATA_WIDTH + j] <= Q[i*DATA_WIDTH + j];
                    default: ;
                    endcase
                end
            end
        end
        
    always @(tag_cell)
    for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
        tag_row[i] = 1;
        for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
             tag_row[i] = tag_row[i] & tag_cell[i*DATA_WIDTH +j];
        end
    end
    
    /*always @ (clk or OutE_C or OutE_R or Q)
        for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
            for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
                if(OutE[i]) Q_out[j] <= Q[i*DATA_WIDTH + j];
            end
        end*/
        
endmodule
