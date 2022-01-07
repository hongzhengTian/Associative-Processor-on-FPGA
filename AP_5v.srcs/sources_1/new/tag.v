module tag_cell
#(parameter DATA_DEPTH = 128)(
tag_A, 
tag_B, 
tag_C, 
tag_F_TSC,
rst, 
clk, 
tag,
tag_TSC
    );
    
    input wire [DATA_DEPTH - 1 : 0] tag_A;
    input wire [DATA_DEPTH - 1 : 0] tag_B;
    input wire [DATA_DEPTH - 1 : 0] tag_C;
    input wire [DATA_DEPTH - 1 : 0] tag_F_TSC;
    input wire clk;
    input wire rst;
    output reg [DATA_DEPTH - 1 : 0] tag;
    output reg [DATA_DEPTH - 1 : 0] tag_TSC;
    
    reg [DATA_DEPTH - 1 : 0] tag_and;
    reg [DATA_DEPTH - 1 : 0] tag_and_TSC;
    
    integer i;
    
    always@(tag_A or tag_B or tag_C or tag_F_TSC)
    for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
        tag_and[i] = tag_A[i] & tag_B[i] & tag_C[i];
        tag_and_TSC[i] = tag_A[i] & tag_F_TSC[i];
    end
    
    always @(posedge clk or negedge rst)
    begin
    for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
        if(!rst)
            begin
                tag[i] <= 0;
                tag_TSC[i] <= 0;
            end
        else
            begin
                tag[i] <= tag_and[i];
                tag_TSC[i] <= tag_and_TSC[i];
            end
    end
    end

    /*always@(tag_A or tag_B or tag_C or tag_F_TSC or negedge rst or posedge clk)
    for (i = 0; i <= DATA_DEPTH - 1; i = i + 1) begin
        if(!rst)
            begin
                tag[i] <= 0;
                tag_TSC[i] <= 0;
            end
        else begin
        tag[i] <= tag_A[i] & tag_B[i] & tag_C[i];
        tag_TSC[i] <= tag_A[i] & tag_F_TSC[i];
        end
    end*/
    
endmodule