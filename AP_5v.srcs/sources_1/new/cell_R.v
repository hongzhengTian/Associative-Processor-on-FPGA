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
input wire [ADDR_WIDTH_CAM - 1 : 0]             addr_input_rbr,
input wire [ADDR_WIDTH_CAM - 1 : 0]             addr_input_cbc,
input wire [ADDR_WIDTH_CAM - 1 : 0]             addr_output_rbr,
input wire [ADDR_WIDTH_CAM - 1 : 0]             addr_output_cbc,
input wire [2 : 0]                              input_mode,
input wire [DATA_WIDTH - 1 : 0]                 input_row,
input wire [DATA_DEPTH - 1 : 0]                 input_col,
input wire [DATA_WIDTH * DATA_DEPTH - 1 : 0]    Q_B,
input wire [DATA_WIDTH * DATA_DEPTH - 1 : 0]    Q_A,
input wire [DATA_DEPTH - 1 : 0]                 Q_S,
input wire                                      abs_opt,
input wire                                      rst_In,
input wire [2:0]                                pass,
input wire [DATA_DEPTH - 1 : 0]                 tag,
input wire [DATA_WIDTH - 1 : 0]                 mask,
input wire                                      clk,
output reg [DATA_WIDTH - 1 : 0]                 Q_out_row,
output reg [DATA_DEPTH - 1 : 0]                 Q_out_col,
output reg [DATA_WIDTH * DATA_DEPTH - 1 : 0]    Q
);

reg [DATA_WIDTH - 1: 0] D [0 : DATA_DEPTH - 1];
reg [DATA_DEPTH - 1 : 0] Ie_R;
reg [DATA_WIDTH - 1 : 0] Ie_C;
reg [DATA_WIDTH - 1: 0] Ie [0 : DATA_DEPTH - 1];
reg [DATA_DEPTH - 1 : 0] OutE_R;
reg [DATA_WIDTH - 1 : 0] OutE_C;
reg [DATA_WIDTH - 1: 0] OutE [0 : DATA_DEPTH - 1];

integer i, j;

always @(*) begin
    case (input_mode)
        RowxRow: begin
            for (j = 0; j < DATA_WIDTH; j = j + 1) begin
                Ie_C[j] = 1'b1;
            end
        end
        ColxCol: begin
            for (i = 0; i < DATA_WIDTH; i = i + 1) begin
                if(!rst_In) begin
                    Ie_C[i] = (addr_input_cbc == i)? 1'b1 : 1'b0;
                end
                else begin
                    Ie_C[i] = 1'b0;
                end
            end
        end
        default: begin
            Ie_C = {{DATA_WIDTH}{1'b0}};
        end
    endcase
end

always @(*) begin
    case (input_mode)
        RowxRow: begin
            for (i = 0; i < DATA_DEPTH; i = i + 1) begin
                if(!rst_In) begin
                    Ie_R[i] = (addr_input_rbr == i)? 1'b1 : 1'b0;
                end
                else begin
                    Ie_R[i] = 1'b0;
                end
            end
        end
        ColxCol: begin
            for (j = 0; j < DATA_DEPTH; j = j + 1) begin
                Ie_R[j] = 1'b1;
            end
        end
        default: begin
            Ie_R = {{DATA_DEPTH}{1'b0}};
        end
    endcase
end

always @(*) begin
    for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
        for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
            Ie[i][j] = Ie_C[j] & Ie_R[i];
        end
    end
end

always @(*) begin
    case (input_mode)
        RowxRow: begin
            for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
                    case({Ie[i][j], abs_opt, (tag[i] & mask[j])})
                        3'b100: D[i][j] = input_row[j];
                        3'b101: D[i][j] = input_row[j];
                        3'b110: D[i][j] = input_row[j];
                        3'b111: D[i][j] = input_row[j];
                        3'b001: begin
                            if ((pass == 1) || (pass == 2)) begin
                                D[i][j] = ~Q_A[i * DATA_WIDTH + j];
                            end
                            else begin
                                D[i][j] = Q_A[i * DATA_WIDTH + j];
                            end
                        end
                        3'b011: begin
                            if(Q_S[i] == 1) begin
                                if ((pass == 2) || (pass == 3)) begin
                                    D[i][j] = ~Q_A[i * DATA_WIDTH + j];
                                end
                                else begin
                                    D[i][j] = Q_A[i * DATA_WIDTH + j];
                                end
                            end
                            else if ((Q_S[i] == 0) || (pass == 1)) begin
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
            for (i = 0; i <= DATA_WIDTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_DEPTH - 1; j = j + 1) begin
                    case({Ie[j][i], abs_opt, (tag[j] & mask[i])})
                    3'b100: D[j][i] = input_col[j];
                    3'b101: D[j][i] = input_col[j];
                    3'b110: D[j][i] = input_col[j];
                    3'b111: D[j][i] = input_col[j];
                    3'b001: begin
                        if ((pass == 1) || (pass == 2)) begin
                            D[j][i] = ~Q_A[j * DATA_WIDTH + i];
                        end
                        else begin
                            D[j][i] = Q_A[j * DATA_WIDTH + i];
                        end
                    end
                    3'b011: begin
                        if(Q_S[j] == 1) begin
                            if ((pass == 2) || (pass == 3)) begin
                                D[j][i] = ~Q_A[j * DATA_WIDTH + i];
                            end
                            else begin
                                D[j][i] = Q_A[j * DATA_WIDTH + i];
                            end
                        end
                        else if ((Q_S[j] == 0) || (pass == 1)) begin
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
                    if(!rst_In) begin
                        D[i][j] = Q_A[i * DATA_WIDTH + j];
                    end
                    else if(abs_opt == 0 && (tag[i] & mask[j]) == 1) begin
                        if ((pass == 1) || (pass == 2)) begin
                            D[i][j] = ~Q_A[i * DATA_WIDTH + j];
                        end
                        else begin
                            D[i][j] = Q_A[i * DATA_WIDTH + j];
                        end
                    end
                    else if (abs_opt == 1 && (tag[i] & mask[j]) == 1) begin
                        if(Q_S[i] == 1) begin
                            if ((pass == 2) || (pass == 3)) begin
                                D[i][j] = ~Q_A[i * DATA_WIDTH + j];
                            end
                            else begin
                                D[i][j] = Q_A[i * DATA_WIDTH + j];
                            end
                        end
                        else if ((Q_S[i] == 0) || (pass == 1)) begin
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
                    if(!rst_In) begin
                        D[i][j] = Q_B[i * DATA_WIDTH + j];
                    end
                    else if(abs_opt == 0 && (tag[i] & mask[j]) == 1) begin
                        if ((pass == 1) || (pass == 2)) begin
                            D[i][j] = ~Q_A[i * DATA_WIDTH + j];
                        end
                        else begin
                            D[i][j] = Q_A[i * DATA_WIDTH + j];
                        end
                    end
                    else if (abs_opt == 1 && (tag[i] & mask[j]) == 1) begin
                        if(Q_S[i] == 1) begin
                            if ((pass == 2) || (pass == 3)) begin
                                D[i][j] = ~Q_A[i * DATA_WIDTH + j];
                            end 
                            else begin
                                D[i][j] = Q_A[i * DATA_WIDTH + j];
                            end
                        end
                        else if ((Q_S[i] == 0) || (pass == 1)) begin
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
            for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
                    if(abs_opt == 0 && (tag[i] & mask[j]) == 1) begin
                        if ((pass == 1) || (pass == 2)) begin
                            D[i][j] = ~Q_A[i * DATA_WIDTH + j];
                        end
                        else begin
                            D[i][j] = Q_A[i * DATA_WIDTH + j];
                        end
                    end
                    else if (abs_opt == 1 && (tag[i] & mask[j]) == 1) begin
                        if(Q_S[i] == 1) begin
                            if ((pass == 2) || (pass == 3)) begin
                                D[i][j] = ~Q_A[i * DATA_WIDTH + j];
                            end
                            else begin
                                D[i][j] = Q_A[i * DATA_WIDTH + j];
                            end
                        end
                        else if ((Q_S[i] == 0) || (pass == 1)) begin
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
            if (addr_output_rbr == DATA_DEPTH + 3) begin
                OutE_C <= 0;
            end
            else begin
                for (j = 0; j < DATA_WIDTH; j = j + 1) begin
                    OutE_C[j] <= 1'b1;
                end
            end
        end
        ColxCol: begin
            for (i = 0; i < DATA_WIDTH; i = i + 1)begin
                OutE_C[i] <= (addr_output_cbc==i)?1'b1:1'b0;
            end
        end
        default: ;
    endcase
end

always @(posedge clk) begin
    case (input_mode)
        RowxRow: begin
            for (i = 0; i < DATA_DEPTH; i = i + 1) begin
                OutE_R[i] <= (addr_output_rbr==i)? 1'b1 : 1'b0;
            end
        end
        ColxCol: begin
            if (addr_output_cbc == DATA_WIDTH + 3)
                begin
                    OutE_R <= 0;
                end
            else begin
                for (j = 0; j < DATA_DEPTH; j = j + 1) 
                    OutE_R[j] <= 1'b1;
            end
        end
        default: ;
    endcase
end

always @(*) begin
    for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
        for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
            OutE[i][j] = OutE_C[j] & OutE_R[i];
        end
    end
end

always @(posedge clk) begin
    case (input_mode)
        RowxRow: begin
            for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_WIDTH - 1; j = j + 1) begin
                    if(OutE[i][j]) begin
                        Q_out_row[j] <= Q[i * DATA_WIDTH + j];
                    end
                end
            end
        end
        ColxCol: begin
            for (i = 0; i <= DATA_WIDTH - 1; i = i + 1) begin
                for (j = 0; j <= DATA_DEPTH - 1; j = j + 1) begin
                    if(OutE[j][i]) begin
                        Q_out_col[j] <= Q[j * DATA_WIDTH + i];
                    end
                end
            end
        end
        default: ;
    endcase
end
        
endmodule

