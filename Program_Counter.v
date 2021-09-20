module program_counter
#(
    parameter ADDR_WIDTH_MEM    = 16,
    parameter ISA_DEPTH         = 64,
    parameter TOTAL_ISA_DEPTH   = 128
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

    always @(posedge clk) begin
        st_cur_ins_cache_delay <= st_cur_ins_cache;
    end

    always @( posedge clk or negedge rst or posedge int)
    begin
        if (!rst)
        begin
            addr_ins <= 0;
            addr_cur_ins <= 0;
        end

        else if (int == 1)
        begin
            addr_ins <= {{1'b1}, {{ADDR_WIDTH_MEM - 1}{1'b0}}};    
        end

        else if (ret_valid == 0)
        begin
            if ((ins_inp_valid == 1) 
             && (addr_ins < TOTAL_ISA_DEPTH) 
             && (ins_cache_rdy == 1) 
             &&(st_cur_ins_cache == START)
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

    /*always@(st_cur_ins_cache_delay)
    begin
        if(st_cur_ins_cache_delay == LOAD_INS)
        begin
            addr_ins = addr_ins - 1;
        end
    end*/

endmodule