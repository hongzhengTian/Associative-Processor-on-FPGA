module cell_R
#(
parameter DATA_WIDTH = 4,
parameter DATA_DEPTH = 4,
parameter ADDR_WIDTH_CAM = 8,
parameter RowxRow = 3'd1,
parameter ColxCol = 3'd2,
parameter COPY_B = 3'd3,
parameter COPY_R = 3'd4,
parameter COPY_A = 3'd5,
parameter RST0 = 3'd6
)
(
input wire [ADDR_WIDTH_CAM - 1 : 0]             addr_input_Row,
input wire [ADDR_WIDTH_CAM - 1 : 0]             addr_input_Col,
input wire [ADDR_WIDTH_CAM - 1 : 0]             addr_output_Row,
input wire [ADDR_WIDTH_CAM - 1 : 0]             addr_output_Col,
input wire [2 : 0]                              input_mode,
input wire [DATA_WIDTH - 1 : 0]                 Ip_row,
input wire [DATA_DEPTH - 1 : 0]                 Ip_col,
input wire [DATA_WIDTH * DATA_DEPTH - 1 : 0]    Q_B,
input wire [DATA_WIDTH * DATA_DEPTH - 1 : 0]    Q_A,
input wire [DATA_DEPTH - 1 : 0]                 Q_S,
input wire                                      abs_opt,
input wire                                      rstIn,
input wire [2:0]                                Pass,
input wire [DATA_DEPTH - 1 : 0]                 tag,
input wire [DATA_WIDTH - 1 : 0]                 Mask,
input wire                                      clk,
output reg [DATA_WIDTH - 1 : 0]                 Q_out_row,
output reg [DATA_DEPTH - 1 : 0]                 Q_out_col,
output reg [DATA_WIDTH * DATA_DEPTH - 1 : 0]    Q
);

reg [DATA_WIDTH - 1: 0] D [0 : DATA_DEPTH - 1];
reg [DATA_DEPTH - 1 : 0] OutE_R;
reg [DATA_WIDTH - 1 : 0] OutE_C;
reg [DATA_DEPTH - 1 : 0] Ie_R;
reg [DATA_WIDTH - 1 : 0] Ie_C;
reg [DATA_WIDTH - 1: 0] Ie [0 : DATA_DEPTH - 1];

integer i, j;
    
always @(*) begin
    case (input_mode)
        RowxRow: begin
            for (j = 0; j < DATA_WIDTH; j = j + 1) begin
                Ie_C[j] = 1'b1;
            end
            for (i = 0; i < DATA_DEPTH; i = i + 1) begin
                if(!rstIn) begin
                    Ie_R[i] = (addr_input_Row == i)? 1'b1 : 1'b0;
                end
                else begin
                    Ie_R[i] = 1'b0;
                end
            end
            for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
                    Ie[i][j] = Ie_C[j] & Ie_R[i];
                end
            end
            for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
                    case({Ie[i][j], abs_opt, (tag[i] & Mask[j])})
                        3'b100: D[i][j] = Ip_row[j];
                        3'b101: D[i][j] = Ip_row[j];
                        3'b110: D[i][j] = Ip_row[j];
                        3'b111: D[i][j] = Ip_row[j];
                        3'b001: begin
                            if ((Pass == 1) || (Pass == 2)) begin
                                D[i][j] = ~Q_A[i * DATA_WIDTH + j];
                            end
                            else begin
                                D[i][j] = Q_A[i * DATA_WIDTH + j];
                            end
                        end
                        3'b011: begin
                            if(Q_S[i] == 1) begin
                                if ((Pass == 2) || (Pass == 3)) begin
                                    D[i][j] = ~Q_A[i * DATA_WIDTH + j];
                                end
                                else begin
                                    D[i][j] = Q_A[i * DATA_WIDTH + j];
                                end
                            end
                            else if ((Q_S[i] == 0) || (Pass == 1)) begin
                                D[i][j] = Q_A[i * DATA_WIDTH + j];
                            end
                            else begin
                                D[i][j] = Q_A[i * DATA_WIDTH + j];
                            end
                        end
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
                    Ie_C[i] = (addr_input_Col == i)? 1'b1 : 1'b0;
                end
                else begin
                    Ie_C[i] = 1'b0;
                end
            end
            for (i = 0; i <= DATA_WIDTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_DEPTH - 1; j = j + 1) begin
                    Ie[j][i] = Ie_C[i] & Ie_R[j];
                end
            end
            for (i = 0; i <= DATA_WIDTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_DEPTH - 1; j = j + 1) begin
                    case({Ie[j][i], abs_opt, (tag[j] & Mask[i])})
                    3'b100: D[j][i] = Ip_col[j];
                    3'b101: D[j][i] = Ip_col[j];
                    3'b110: D[j][i] = Ip_col[j];
                    3'b111: D[j][i] = Ip_col[j];
                    3'b001: begin
                        if ((Pass == 1) || (Pass == 2)) begin
                            D[j][i] = ~Q_A[j * DATA_WIDTH + i];
                        end
                        else begin
                            D[j][i] = Q_A[j * DATA_WIDTH + i];
                        end
                    end
                    3'b011: begin
                        if(Q_S[j] == 1) begin
                            if ((Pass == 2) || (Pass == 3)) begin
                                D[j][i] = ~Q_A[j * DATA_WIDTH + i];
                            end
                            else begin
                                D[j][i] = Q_A[j * DATA_WIDTH + i];
                            end
                        end
                        else if ((Q_S[j] == 0) || (Pass == 1)) begin
                            D[j][i] = Q_A[j * DATA_WIDTH + i];
                        end
                        else begin
                            D[j][i] = Q_A[j * DATA_WIDTH + i];
                        end
                    end
                    default: D[j][i] = Q[j * DATA_WIDTH + i];
                    endcase
                end
            end
        end
        COPY_A: begin
            for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
                    if(!rstIn) begin
                        D[i][j] = Q_A[i * DATA_WIDTH + j];
                    end
                    else if(abs_opt == 0 && (tag[i] & Mask[j]) == 1) begin
                        if ((Pass == 1) || (Pass == 2)) begin
                            D[i][j] = ~Q_A[i * DATA_WIDTH + j];
                        end
                        else begin
                            D[i][j] = Q_A[i * DATA_WIDTH + j];
                        end
                    end
                    else if (abs_opt == 1 && (tag[i] & Mask[j]) == 1) begin
                        if(Q_S[i] == 1) begin
                            if ((Pass == 2) || (Pass == 3)) begin
                                D[i][j] = ~Q_A[i * DATA_WIDTH + j];
                            end
                            else begin
                                D[i][j] = Q_A[i * DATA_WIDTH + j];
                            end
                        end
                        else if ((Q_S[i] == 0) || (Pass == 1)) begin
                            D[i][j] = Q_A[i * DATA_WIDTH + j];
                        end
                        else begin
                            D[i][j] = Q_A[i * DATA_WIDTH + j];
                        end
                    end
                    else begin
                        D[i][j] = Q[i * DATA_WIDTH + j];
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
                    else if(abs_opt == 0 && (tag[i] & Mask[j]) == 1) begin
                        if ((Pass == 1) || (Pass == 2)) begin
                            D[i][j] = ~Q_A[i * DATA_WIDTH + j];
                        end
                        else begin
                            D[i][j] = Q_A[i * DATA_WIDTH + j];
                        end
                    end
                    else if (abs_opt == 1 && (tag[i] & Mask[j]) == 1) begin
                        if(Q_S[i] == 1) begin
                            if ((Pass == 2) || (Pass == 3)) begin
                                D[i][j] = ~Q_A[i * DATA_WIDTH + j];
                            end 
                            else begin
                                D[i][j] = Q_A[i * DATA_WIDTH + j];
                            end
                        end
                        else if ((Q_S[i] == 0) || (Pass == 1)) begin
                            D[i][j] = Q_A[i * DATA_WIDTH + j];
                        end
                        else begin
                            D[i][j] = Q_A[i * DATA_WIDTH + j];
                        end
                    end
                    else begin
                        D[i][j] = Q[i * DATA_WIDTH + j];
                    end
                end
            end
        end
        RST0: begin
            for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
                D[i] = 0;
            end
        end
        default: begin
            Ie_C = {{DATA_WIDTH}{1'b0}};
            Ie_R = {{DATA_DEPTH}{1'b0}};
            for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
                    if(abs_opt == 0 && (tag[i] & Mask[j]) == 1) begin
                        if ((Pass == 1) || (Pass == 2)) begin
                            D[i][j] = ~Q_A[i * DATA_WIDTH + j];
                        end
                        else begin
                            D[i][j] = Q_A[i * DATA_WIDTH + j];
                        end
                    end
                    else if (abs_opt == 1 && (tag[i] & Mask[j]) == 1) begin
                        if(Q_S[i] == 1) begin
                            if ((Pass == 2) || (Pass == 3)) begin
                                D[i][j] = ~Q_A[i * DATA_WIDTH + j];
                            end
                            else begin
                                D[i][j] = Q_A[i * DATA_WIDTH + j];
                            end
                        end
                        else if ((Q_S[i] == 0) || (Pass == 1)) begin
                            D[i][j] = Q_A[i * DATA_WIDTH + j];
                        end
                        else begin
                            D[i][j] = Q_A[i * DATA_WIDTH + j];
                        end
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
            for (i = 0; i < DATA_DEPTH; i = i + 1) begin
                OutE_R[i] <= (addr_output_Row==i)? 1'b1 : 1'b0;
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
            if (addr_output_Col == DATA_WIDTH + 3)
                begin
                    OutE_R <= 0;
                end
            else begin
                for (j = 0; j < DATA_DEPTH; j = j + 1) 
                    OutE_R[j] <= 1'b1;
            end
            for (i = 0; i < DATA_WIDTH; i = i + 1)begin
                OutE_C[i] <= (addr_output_Col==i)?1'b1:1'b0;
            end
            for (i = 0; i <= DATA_WIDTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_DEPTH - 1; j = j + 1) begin
                    if((OutE_R[j]&OutE_C[i] == 1)) begin
                        Q_out_col[j] <= Q[j * DATA_WIDTH + i];
                    end
                end
            end
        end
        default: ;
    endcase
end
        
endmodule

