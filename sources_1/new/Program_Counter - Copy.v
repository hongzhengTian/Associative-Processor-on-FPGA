

module program_counter
#(
    parameter ADDR_WIDTH_MEM    = 16,
    parameter ISA_DEPTH         = 64,
    parameter TOTAL_ISA_DEPTH   = 128,
    parameter DDR_ADDR_WIDTH    = 28
)
(
    /* the interface of system signal */
    input wire                              clk,
    input wire                              rst,
    input wire                              ret_valid,
    input wire                              int,

    /* the interface of AP_ctrl */
    input wire                              ins_inp_valid,
    input wire [ADDR_WIDTH_MEM - 1 : 0]     ret_addr_pc,
    input wire [DDR_ADDR_WIDTH - 1 : 0]     jmp_addr_pc,
    output reg [ADDR_WIDTH_MEM - 1 : 0]     addr_cur_ins,

    /* the interface of instruction cache */
    output reg [ADDR_WIDTH_MEM - 1 : 0]     addr_ins,
    input wire                              ins_cache_rdy,
    input wire [3 : 0]                      st_cur_ins_cache,
    input wire [9 : 0]                      load_times
);
    integer i;
    localparam                              START           = 4'd1;
    localparam                              LOAD_INS        = 4'd2;
    localparam                              SENT_INS        = 4'd3;
    reg [3 : 0] st_cur_ins_cache_delay;
    reg         int_set;

    wire [ADDR_WIDTH_MEM - 1 : 0] jmp_addr_pc_short;
    assign jmp_addr_pc_short = jmp_addr_pc [ADDR_WIDTH_MEM - 1 : 0];

    always @(posedge clk) begin
        st_cur_ins_cache_delay <= st_cur_ins_cache;
    end

    always @(posedge int or negedge rst)
    begin
        if (!rst)
        begin
            int_set <= 0;
        end
        else if (int == 1)
        begin
            int_set <= 1;    
        end
    end

    always @( posedge clk or negedge rst or posedge int_set)
    begin
        if (!rst)
        begin
            addr_ins <= 0;
            addr_cur_ins <= 0;
        end

        else if (int_set == 1)
        begin
            if (ins_inp_valid == 0)
                addr_ins <= {{1'b1}, {{ADDR_WIDTH_MEM - 1}{1'b0}}};
            else addr_ins <= jmp_addr_pc_short / 8; 
        end

        else if (ret_valid == 0)
        begin
            if ((ins_inp_valid == 1) 
             && (addr_ins < TOTAL_ISA_DEPTH) 
             && (ins_cache_rdy == 1) 
             &&(st_cur_ins_cache == SENT_INS)
             &&(addr_ins != ISA_DEPTH * load_times))
            begin
                addr_ins <= addr_ins + 1;
                addr_cur_ins <= addr_ins + 1;
            end

            else
            begin
                addr_ins <= addr_ins;
            end
        end

        
        else if (ret_valid == 1)
            begin
                for(i = 0; i <= ADDR_WIDTH_MEM - 1; i = i + 1)
                begin
                    addr_ins[i] <= ret_addr_pc[i];
                end
            end

        else
        begin
            addr_ins <= addr_ins;
        end
    end

endmodule