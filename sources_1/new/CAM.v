module CAM
  #(parameter DATA_WIDTH = 4,
  parameter DATA_DEPTH = 4,
  parameter ADDR_WIDTH_CAM = 8,
  parameter RowxRow = 3'd1,
  parameter ColxCol = 3'd2,
  parameter COPY_B = 3'd3,
  parameter COPY_R = 3'd4,
  parameter COPY_A = 3'd5
  )
    (
    input wire clk,
    input wire [2 : 0]input_mode,
    input wire [DATA_WIDTH - 1 : 0] Mask_A,
    input wire [DATA_WIDTH - 1 : 0] Mask_B,
    input wire [DATA_WIDTH - 1 : 0] Mask_R,
    input wire [2:0]Pass,
    input wire rst_tag,
    input wire ABS_opt,

    input wire [ADDR_WIDTH_CAM - 1 : 0]addr_input_Row_A,
    input wire [ADDR_WIDTH_CAM - 1 : 0]addr_input_Col_A,
    input wire rstInA,
    input wire [ADDR_WIDTH_CAM - 1 : 0]addr_output_Row_A,
    input wire [ADDR_WIDTH_CAM - 1 : 0]addr_output_Col_A,
    input wire [DATA_WIDTH - 1 : 0] Input_A_row,
    input wire [DATA_DEPTH - 1 : 0] Input_A_col,
    input wire Key_A,

    input wire [ADDR_WIDTH_CAM - 1 : 0]addr_input_Row_B,
    input wire [ADDR_WIDTH_CAM - 1 : 0]addr_input_Col_B,
    input wire rstInB,
    input wire [ADDR_WIDTH_CAM - 1 : 0]addr_output_Row_B,
    input wire [ADDR_WIDTH_CAM - 1 : 0]addr_output_Col_B,
    input wire [DATA_WIDTH - 1 : 0] Input_B_row,
    input wire [DATA_DEPTH - 1 : 0] Input_B_col,
    input wire Key_B,
    
    input wire [ADDR_WIDTH_CAM - 1 : 0]addr_input_Row_R,
    input wire [ADDR_WIDTH_CAM - 1 : 0]addr_input_Col_R,
    input wire rstInR,
    input wire [ADDR_WIDTH_CAM - 1 : 0]addr_output_Row_R,
    input wire [ADDR_WIDTH_CAM - 1 : 0]addr_output_Col_R,
    input wire [DATA_WIDTH - 1 : 0] Input_R_row,
    input wire [DATA_DEPTH - 1 : 0] Input_R_col,

    input wire rstInC,
    input wire [DATA_DEPTH - 1 : 0] Input_C,
    input wire Key_C,
    input wire Mask_C,
    
    input wire rstInF,
    input wire [DATA_DEPTH - 1 : 0] Input_F,
    input wire Key_F,
    input wire Mask_F,

    output wire [DATA_WIDTH - 1 : 0] Q_out_A_row,
    output wire [DATA_DEPTH - 1 : 0] Q_out_A_col,
    output wire [DATA_WIDTH - 1 : 0] Q_out_R_row,
    output wire [DATA_DEPTH - 1 : 0] Q_out_R_col,
    output wire [DATA_WIDTH - 1 : 0] Q_out_B_row,
    output wire [DATA_DEPTH - 1 : 0] Q_out_B_col,
    output wire [DATA_DEPTH - 1 : 0] Q_out_F,
    output wire [DATA_DEPTH - 1 : 0] Q_out_C
    );
    
    wire [DATA_WIDTH * DATA_DEPTH - 1 : 0] Q_A;
    wire [DATA_WIDTH * DATA_DEPTH - 1 : 0] Q_R;
    wire [DATA_WIDTH * DATA_DEPTH - 1 : 0] Q_B;
    wire [DATA_DEPTH - 1 : 0] Q_S;
    wire [DATA_DEPTH - 1 : 0] tag_row_A;
    wire [DATA_DEPTH - 1 : 0] tag_row_B;  
    wire [DATA_DEPTH - 1 : 0] tag_row_C;
    wire [DATA_DEPTH - 1 : 0] tag_row_F;
    wire [DATA_DEPTH - 1 : 0] tag_out;
    wire [DATA_DEPTH - 1 : 0] tag_TSC;
    
cell_A #(
DATA_WIDTH, 
DATA_DEPTH, 
ADDR_WIDTH_CAM,
RowxRow,
ColxCol,
COPY_B,
COPY_R,
COPY_A
) cell_A(
.Ip_row(Input_A_row),
.Ip_col(Input_A_col),
.Q_R(Q_R),
.Q_B(Q_B),
.addr_input_Row(addr_input_Row_A),
.addr_input_Col(addr_input_Col_A),
.input_mode(input_mode),
.rstIn(rstInA),
.Key(Key_A),
.Mask(Mask_A),
.clk(clk),
.addr_output_Row(addr_output_Row_A),
.addr_output_Col(addr_output_Col_A),
.Q_out_row(Q_out_A_row),
.Q_out_col(Q_out_A_col),
.tag_row(tag_row_A),
.Q(Q_A),
.Q_S(Q_S)
    );
    
cell_B #(
DATA_WIDTH, 
DATA_DEPTH, 
ADDR_WIDTH_CAM,
RowxRow,
ColxCol,
COPY_B,
COPY_R,
COPY_A
) cell_B(
.Ip_row(Input_B_row),
.Ip_col(Input_B_col),
.Q_R(Q_R),
.Q_A(Q_A),
.addr_input_Row(addr_input_Row_B),
.addr_input_Col(addr_input_Col_B),
.input_mode(input_mode),
.rstIn(rstInB),
.Key(Key_B),
.Mask(Mask_B),
.clk(clk),
.tag(tag_out),
.addr_output_Row(addr_output_Row_B),
.addr_output_Col(addr_output_Col_B),

.Q_out_row(Q_out_B_row),
.Q_out_col(Q_out_B_col),
.tag_row(tag_row_B),
.Q(Q_B)
    );
    
cell_C #(DATA_DEPTH) cell_C(
//.addr_input(addr_C), 
.rstIn(rstInC),
.Ip(Input_C), 
.Key(Key_C), 
.Mask(Mask_C), 
.Pass(Pass),
.clk(clk), 
.tag(tag_out),
.Q(Q_out_C),
.tag_cell(tag_row_C));
    
tag_cell #(DATA_DEPTH) tag_cell(
.tag_A(tag_row_A), 
.tag_B(tag_row_B), 
.tag_C(tag_row_C), 
.tag_F_TSC(tag_row_F),
.rst(rst_tag), 
.clk(clk), 
.tag(tag_out),
.tag_TSC(tag_TSC));

cell_F #(DATA_DEPTH) cell_F(
//.addr_input(addr_F),
.Ip(Input_F),
.Key(Key_F),
.Mask(Mask_F),
.Pass(Pass),
.rstIn(rstInF),
.Q_S(Q_S),
.Q(Q_out_F),
.ABS_opt(ABS_opt),
.clk(clk),
.tag_cell(tag_row_F),
.tag(tag_TSC));

cell_R #(
DATA_WIDTH, 
DATA_DEPTH, 
ADDR_WIDTH_CAM,
RowxRow,
ColxCol,
COPY_B,
COPY_R,
COPY_A) cell_R(
.addr_input_Row(addr_input_Row_R),
.addr_input_Col(addr_input_Col_R),
.addr_output_Row(addr_output_Row_R),
.addr_output_Col(addr_output_Col_R),
.input_mode(input_mode),
.Ip_row(Input_R_row),
.Ip_col(Input_R_col),
.Q_B(Q_B),
.Q_A(Q_A),
.Q_S(Q_S),
.ABS_opt(ABS_opt),
.rstIn(rstInR),
.Pass(Pass),
.tag(tag_TSC),
.Mask(Mask_R),
.clk(clk),
.Q_out_row(Q_out_R_row),
.Q_out_col(Q_out_R_col),
.Q(Q_R)
);
    
endmodule
