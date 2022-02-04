module cell_A
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
input wire [DATA_WIDTH - 1 : 0]                 Ip_row,
input wire [DATA_DEPTH - 1 : 0]                 Ip_col,
input wire [DATA_WIDTH * DATA_DEPTH - 1 : 0]    Q_R,
input wire [DATA_WIDTH * DATA_DEPTH - 1 : 0]    Q_B,
input wire [ADDR_WIDTH_CAM - 1 : 0]             addr_input_Row,
input wire [ADDR_WIDTH_CAM - 1 : 0]             addr_input_Col,
input wire [2 : 0]                              input_mode,
input wire                                      rstIn,
input wire                                      Key,
input wire [DATA_WIDTH - 1 : 0]                 Mask,
input wire                                      clk,
input wire [ADDR_WIDTH_CAM - 1 : 0]             addr_output_Row,
input wire [ADDR_WIDTH_CAM - 1 : 0]             addr_output_Col,

output reg [DATA_WIDTH - 1 : 0]                 Q_out_row,
output reg [DATA_DEPTH - 1 : 0]                 Q_out_col,
output reg [DATA_DEPTH - 1 : 0]                 tag_row,
output reg [DATA_WIDTH * DATA_DEPTH - 1 : 0]    Q,
output reg [DATA_DEPTH - 1 : 0]                 Q_S
);

reg [DATA_DEPTH - 1 : 0] Ie_R;
reg [DATA_WIDTH - 1 : 0] Ie_C;
reg [DATA_DEPTH - 1 : 0] OutE_R;
reg [DATA_WIDTH - 1 : 0] OutE_C;
reg [DATA_WIDTH * DATA_DEPTH - 1 : 0] tag_cell;

reg [DATA_WIDTH - 1: 0] D [0 : DATA_DEPTH - 1]; 
reg [DATA_WIDTH * DATA_DEPTH - 1 : 0] Qb;


integer i, j;

always @(*) begin
    case (input_mode)
        RowxRow: begin
            for (j = 0; j < DATA_WIDTH; j = j + 1) begin
                Ie_C[j] = 1'b1;
            end
            for (i = 0; i < DATA_DEPTH; i = i + 1) begin
                if(!rstIn) begin
                    Ie_R[i] = (addr_input_Row==i)? 1'b1 : 1'b0;
                end
                else begin
                    Ie_R[i] = 1'b0;
                end
            end
            for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
                    case ((Ie_R[i] & Ie_C[j]))
                        1'b0: D[i][j] = Q[i * DATA_WIDTH + j];
                        1'b1: D[i][j] = Ip_row[j];
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
                    if(!rstIn) begin
                        Ie_C[i] = (addr_input_Col==i)? 1'b1 : 1'b0;
                    end
                    else begin
                        Ie_C[i] = 1'b0;
                    end
                end
            for (i = 0; i <= DATA_WIDTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_DEPTH - 1; j = j + 1) begin
                    begin
                        case ((Ie_R[j]&Ie_C[i]))
                            1'b0: D[j][i] = Q[j * DATA_WIDTH + i];
                            1'b1: D[j][i] = Ip_col[j];
                            default: D[j][i] = Q[j * DATA_WIDTH + i];
                        endcase
                    end
                end
            end
        end
        COPY_B: begin
            for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
                    if(!rstIn) begin
                        D[i][j] = Q_B[i * DATA_WIDTH + j];
                    end
                    else begin
                        D[i][j] = Q[i * DATA_WIDTH + j];
                    end
                end
            end
        end
        COPY_R: begin
            for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
                    if(!rstIn) begin
                        D[i][j] = Q_R[i * DATA_WIDTH + j];
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
                    D[i][j] = Q[i * DATA_WIDTH + j];
                end
            end
        end
    endcase
end
    
always @(posedge clk) begin
    for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
        Q_S[i] <= D[i][DATA_WIDTH - 1]; 
        for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
            Q[i * DATA_WIDTH + j] <= D[i][j];
            Qb[i * DATA_WIDTH + j] <= ~D[i][j];
        end
    end
end
    
always @(posedge clk) begin
    case (input_mode)
        RowxRow: begin
            if (addr_output_Row == DATA_DEPTH + 3) begin
                OutE_C <= 0;
            end
            else begin
                for (j = 0; j < DATA_WIDTH; j = j + 1) begin
                    OutE_C[j] <= 1'b1;
                end
            end
            for (i = 0; i < DATA_DEPTH; i = i + 1)begin
                OutE_R[i] <= (addr_output_Row == i)? 1'b1 : 1'b0;
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
            if (addr_output_Col == DATA_WIDTH + 3) begin
                OutE_R <= 0;
            end
            else begin
                for (j = 0; j < DATA_DEPTH; j = j + 1) begin
                    OutE_R[j] <= 1'b1;
                end
            end
            for (i = 0; i < DATA_WIDTH; i = i + 1)begin
                OutE_C[i] <= (addr_output_Col == i)? 1'b1 : 1'b0;
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
        
always @(Mask or Key or clk or Q or Qb) begin
    for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
        for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
            case ({Mask[j],Key})
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

