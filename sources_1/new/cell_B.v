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
input wire [DATA_WIDTH - 1 : 0]               input_row,
input wire [DATA_DEPTH - 1 : 0]               input_col,
input wire [DATA_WIDTH * DATA_DEPTH - 1 : 0]  Q_R,
input wire [DATA_WIDTH * DATA_DEPTH - 1 : 0]  Q_A,
input wire [ADDR_WIDTH_CAM - 1 : 0]           addr_input_rbr,
input wire [ADDR_WIDTH_CAM - 1 : 0]           addr_input_cbc,
input wire [2 : 0]                            input_mode,
input wire                                    rst_In,
input wire                                    key,
input wire [DATA_WIDTH - 1 : 0]               mask,
input wire                                    clk,
input wire [DATA_DEPTH - 1 : 0]               tag,
input wire [ADDR_WIDTH_CAM - 1 : 0]           addr_output_rbr,
input wire [ADDR_WIDTH_CAM - 1 : 0]           addr_output_cbc,
  
output reg [DATA_WIDTH - 1 : 0]               Q_out_row,
output reg [DATA_DEPTH - 1 : 0]               Q_out_col,
output reg [DATA_DEPTH - 1 : 0]               tag_row,
output reg [DATA_WIDTH * DATA_DEPTH - 1 : 0]  Q
);

reg [DATA_DEPTH - 1 : 0] OutE_R;
reg [DATA_WIDTH - 1 : 0] OutE_C;
reg [DATA_DEPTH - 1 : 0] Ie_R;
reg [DATA_WIDTH - 1 : 0] Ie_C;
reg [DATA_WIDTH * DATA_DEPTH - 1 : 0] tag_cell;
reg [DATA_WIDTH * DATA_DEPTH - 1 : 0] Qb; 
reg [DATA_WIDTH - 1: 0] D [0 : DATA_DEPTH - 1]; 

integer i, j;

always @(*) begin
    case (input_mode)
        RowxRow: begin
            for (j = 0; j < DATA_WIDTH; j = j + 1) begin
                Ie_C[j] = 1'b1;
            end
            for (i = 0; i < DATA_DEPTH; i = i + 1) begin
                if (!rst_In) begin
                    Ie_R[i] = (addr_input_rbr == i)? 1'b1 : 1'b0;
                end
                else begin
                    Ie_R[i] = 1'b0;
                end
            end
            for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
                    case({tag[i], mask[j], (Ie_R[i] & Ie_C[j])})
                        3'b001: D[i][j] = input_row[j];
                        3'b011: D[i][j] = input_row[j];
                        3'b101: D[i][j] = input_row[j];
                        3'b111: D[i][j] = input_row[j];
                        3'b110: D[i][j] = Qb[i * DATA_WIDTH + j];
                        default: D[i][j] = Q[i * DATA_WIDTH + j];
                    endcase
                end
            end
        end
        ColxCol: begin
            for (j = 0; j < DATA_DEPTH; j = j + 1) begin
                Ie_R[j] = 1'b1;
            end
            for (i = 0; i < DATA_WIDTH; i = i + 1) begin
                if (!rst_In) begin
                    Ie_C[i] = (addr_input_cbc == i)? 1'b1 : 1'b0;
                end
                else begin
                    Ie_C[i] = 1'b0;
                end
            end
            for (i = 0; i <= DATA_WIDTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_DEPTH - 1; j = j + 1) begin
                    case({tag[j], mask[i], (Ie_R[j] & Ie_C[i])})
                        3'b001: D[j][i] = input_col[j];
                        3'b011: D[j][i] = input_col[j];
                        3'b101: D[j][i] = input_col[j];
                        3'b111: D[j][i] = input_col[j];
                        3'b110: D[j][i] = Qb[j * DATA_WIDTH + i];
                        default: D[j][i] = Q[j * DATA_WIDTH + i];
                    endcase
                end
            end
        end
        COPY_A: begin
            for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
                    if(!rst_In) begin
                        D[i][j] = Q_A[i * DATA_WIDTH + j];
                    end
                    else begin
                        if (tag[i] == 1 && mask[j] == 1 ) begin
                            D[i][j] = Qb[i * DATA_WIDTH + j];
                        end
                        else begin
                            D[i][j] = Q[i * DATA_WIDTH + j];
                        end
                    end
                end
            end
         end
        COPY_R: begin
            for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
                    if(!rst_In) begin
                        D[i][j] = Q_R[i * DATA_WIDTH + j];
                    end
                    else if (tag[i] == 1 && mask[j] == 1 ) begin
                        D[i][j] = Qb[i * DATA_WIDTH + j];
                    end 
                    else begin
                        D[i][j] = Q[i * DATA_WIDTH + j];
                    end
                end
            end
        end
        default: begin
          Ie_C = {{DATA_WIDTH}{1'b0}};
          Ie_R = {{DATA_DEPTH}{1'b0}};
          for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
                    if (tag[i] == 1 && mask[j] == 1 ) begin
                        D[i][j] = Qb[i * DATA_WIDTH + j];
                    end
                    else begin
                        D[i][j] = Q[i * DATA_WIDTH + j];
                    end
                end
            end
        end
    endcase
end

always@(posedge clk) begin
    for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
        for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
            Q[i * DATA_WIDTH+j] <= D[i][j];
            Qb[i * DATA_WIDTH+j] <= ~D[i][j];
        end
    end
end

always @(posedge clk) begin
    case (input_mode)
        RowxRow: begin
            if (addr_output_rbr == DATA_DEPTH + 3) begin
                OutE_C <= 0;
            end
            else begin
                for (j = 0; j < DATA_WIDTH; j = j + 1) begin
                    OutE_C[j] <= 1'b1;
                end
            end
            for (i = 0; i < DATA_DEPTH; i = i + 1) begin
                OutE_R[i] <= (addr_output_rbr == i)? 1'b1 : 1'b0;
            end
            for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
                    if((OutE_R[i] & OutE_C[j] == 1)) begin
                        Q_out_row[j] <= Q[i * DATA_WIDTH + j];
                    end
                end
            end
        end
        ColxCol: begin
            if (addr_output_cbc == DATA_WIDTH + 3) begin
                OutE_R <= 0;
            end
            else begin
                for (j = 0; j < DATA_DEPTH; j = j + 1) begin
                    OutE_R[j] <= 1'b1;
                end
            end
            for (i = 0; i < DATA_WIDTH; i = i + 1) begin
                OutE_C[i] <= (addr_output_cbc == i)? 1'b1 : 1'b0;
            end
            for (i = 0; i <= DATA_WIDTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_DEPTH - 1; j = j + 1) begin
                    if((OutE_R[j] & OutE_C[i] == 1)) begin
                        Q_out_col[j] <= Q[j * DATA_WIDTH + i];
                    end
                end
            end
        end
        default: ;
    endcase
end
    
always @(mask or key or clk or Q or Qb) begin
    for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
        for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
            case ({mask[j], key})
                2'b00: tag_cell[i * DATA_WIDTH + j] = 1'b1;
                2'b01: tag_cell[i * DATA_WIDTH + j] = 1'b1;
                2'b10: tag_cell[i * DATA_WIDTH + j] = Qb[i * DATA_WIDTH + j];
                2'b11: tag_cell[i * DATA_WIDTH + j] = Q[i * DATA_WIDTH + j];
                default: ;
            endcase
        end
    end
end

always @(tag_cell) begin
    for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
        tag_row[i] = 1;
        for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
            tag_row[i] = tag_row[i] & tag_cell[i * DATA_WIDTH +j];
        end
    end
end
endmodule
