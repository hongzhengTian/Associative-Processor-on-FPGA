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
    input wire [DATA_WIDTH - 1 : 0] mask_A,
    input wire [DATA_WIDTH - 1 : 0] mask_B,
    input wire [DATA_WIDTH - 1 : 0] mask_R,
    input wire [2:0]pass,
    input wire rst_tag,
    input wire abs_opt,

    input wire [ADDR_WIDTH_CAM - 1 : 0]addr_input_rbr_A,
    input wire [ADDR_WIDTH_CAM - 1 : 0]addr_input_cbc_A,
    input wire rst_InA,
    input wire [ADDR_WIDTH_CAM - 1 : 0]addr_output_rbr_A,
    input wire [ADDR_WIDTH_CAM - 1 : 0]addr_output_cbc_A,
    input wire [DATA_WIDTH - 1 : 0] input_A_row,
    input wire [DATA_DEPTH - 1 : 0] input_A_col,
    input wire key_A,

    input wire [ADDR_WIDTH_CAM - 1 : 0]addr_input_rbr_B,
    input wire [ADDR_WIDTH_CAM - 1 : 0]addr_input_cbc_B,
    input wire rst_InB,
    input wire [ADDR_WIDTH_CAM - 1 : 0]addr_output_rbr_B,
    input wire [ADDR_WIDTH_CAM - 1 : 0]addr_output_cbc_B,
    input wire [DATA_WIDTH - 1 : 0] input_B_row,
    input wire [DATA_DEPTH - 1 : 0] input_B_col,
    input wire key_B,
    
    input wire [ADDR_WIDTH_CAM - 1 : 0]addr_input_rbr_R,
    input wire [ADDR_WIDTH_CAM - 1 : 0]addr_input_cbc_R,
    input wire rst_InR,
    input wire [ADDR_WIDTH_CAM - 1 : 0]addr_output_rbr_R,
    input wire [ADDR_WIDTH_CAM - 1 : 0]addr_output_cbc_R,
    input wire [DATA_WIDTH - 1 : 0] input_R_row,
    input wire [DATA_DEPTH - 1 : 0] input_R_col,

    input wire rst_InC,
    input wire [DATA_DEPTH - 1 : 0] input_C,
    input wire key_C,
    
    input wire rst_InF,
    input wire [DATA_DEPTH - 1 : 0] input_F,
    input wire key_F,

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
.input_row(input_A_row),
.input_col(input_A_col),
.Q_R(Q_R),
.Q_B(Q_B),
.addr_input_rbr(addr_input_rbr_A),
.addr_input_cbc(addr_input_cbc_A),
.input_mode(input_mode),
.rst_In(rst_InA),
.key(key_A),
.mask(mask_A),
.clk(clk),
.addr_output_rbr(addr_output_rbr_A),
.addr_output_cbc(addr_output_cbc_A),
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
.input_row(input_B_row),
.input_col(input_B_col),
.Q_R(Q_R),
.Q_A(Q_A),
.addr_input_rbr(addr_input_rbr_B),
.addr_input_cbc(addr_input_cbc_B),
.input_mode(input_mode),
.rst_In(rst_InB),
.key(key_B),
.mask(mask_B),
.clk(clk),
.tag(tag_out),
.addr_output_rbr(addr_output_rbr_B),
.addr_output_cbc(addr_output_cbc_B),

.Q_out_row(Q_out_B_row),
.Q_out_col(Q_out_B_col),
.tag_row(tag_row_B),
.Q(Q_B)
    );
    
cell_C #(DATA_DEPTH) cell_C(
//.addr_input(addr_C), 
.rst_In(rst_InC),
.input_C(input_C), 
.key(key_C), 
.pass(pass),
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
.input_F(input_F),
.key(key_F),
.pass(pass),
.rst_In(rst_InF),
.Q_S(Q_S),
.Q(Q_out_F),
.abs_opt(abs_opt),
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
.addr_input_rbr(addr_input_rbr_R),
.addr_input_cbc(addr_input_cbc_R),
.addr_output_rbr(addr_output_rbr_R),
.addr_output_cbc(addr_output_cbc_R),
.input_mode(input_mode),
.input_row(input_R_row),
.input_col(input_R_col),
.Q_B(Q_B),
.Q_A(Q_A),
.Q_S(Q_S),
.abs_opt(abs_opt),
.rst_In(rst_InR),
.pass(pass),
.tag(tag_TSC),
.mask(mask_R),
.clk(clk),
.Q_out_row(Q_out_R_row),
.Q_out_col(Q_out_R_col),
.Q(Q_R)
);
    
endmodule
