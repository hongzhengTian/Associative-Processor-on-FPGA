module sm_data_cache
#(
    parameter DATA_CACHE_DEPTH  = 16,
    parameter DATA_WIDTH        = 16,
    parameter DATA_DEPTH        = 16,
    parameter DDR_ADDR_WIDTH    = 28,
    parameter ADDR_WIDTH_MEM    = 16,
    parameter ADDR_WIDTH_CAM    = 8
)
(
    /* the interface of system signal */
    input wire                              clk,
    input wire                              rst,
    input wire                              int_set,

    /* the interface of AP_ctrl */
    input wire [2 : 0]                      data_cmd,
    input wire                              store_ddr_en,

    /* the interface of data cache's ALU */
    input wire                              dc_exp_2,
    input wire                              dc_exp_3,
    input wire                              dc_exp_5,
    input wire                              dc_exp_7,

    output reg [3 : 0]                      st_cur

);
/* states */
localparam                                  START_PRE       = 4'd0;    
localparam                                  START           = 4'd1;
localparam                                  LOAD_DATA       = 4'd2;
localparam                                  STORE_DATA      = 4'd3;
localparam                                  SENT_DATA_RBR   = 4'd4;
localparam                                  SENT_DATA_CBC   = 4'd5;
localparam                                  GET_DATA_RBR    = 4'd6;
localparam                                  GET_DATA_CBC    = 4'd7;
localparam                                  SENT_ADDR       = 4'd8;
localparam                                  LOAD_JMP_ADDR   = 4'd9;
localparam                                  STORE_DATA_END  = 4'd10;

/* data_cache command */
localparam                                  RowxRow_load    = 3'd1;
localparam                                  RowxRow_store   = 3'd2;
localparam                                  ColxCol_load    = 3'd3;
localparam                                  ColxCol_store   = 3'd4;
localparam                                  Addr_load       = 3'd5;

reg [3 : 0]                                 st_next;

/* state machine */
always @(posedge clk or negedge rst) begin
    if (!rst) begin
        st_cur <= START_PRE;
    end
    else begin
        st_cur <= st_next;
    end    
end

/* state machine */
always @(*) begin
    case (st_cur)
        START_PRE: begin
            st_next = START;
        end
        START: begin
            case ({data_cmd, store_ddr_en})
                {RowxRow_load, 1'b1}: st_next = STORE_DATA;
                {RowxRow_load, 1'b0}: st_next = SENT_DATA_RBR;
                {RowxRow_store, 1'b1}: st_next = STORE_DATA;
                {RowxRow_store, 1'b0}: st_next = GET_DATA_RBR;
                {ColxCol_load, 1'b1}: st_next = STORE_DATA;
                {ColxCol_load, 1'b0}: st_next = SENT_DATA_CBC;
                {Addr_load, 1'b1}: st_next = STORE_DATA;
                {Addr_load, 1'b0}: st_next = SENT_ADDR;
                {ColxCol_store, 1'b1}: st_next = STORE_DATA;
                {ColxCol_store, 1'b0}: st_next = GET_DATA_CBC;
                2'b01: st_next = STORE_DATA;
                2'b00: st_next = START_PRE;
                default: st_next = START_PRE;
            endcase
        end
        SENT_ADDR: begin
            case (dc_exp_2)
                1'b1: st_next = START_PRE; 
                default: st_next = SENT_ADDR;
            endcase
        end
        SENT_DATA_RBR: begin 
            case ({dc_exp_3, int_set})
                2'b10: st_next = START_PRE;
                2'b00: st_next = LOAD_DATA;
                2'b01: st_next = GET_DATA_CBC;
                2'b11: st_next = GET_DATA_CBC;
                default: st_next = SENT_DATA_RBR;
            endcase
        end
        SENT_DATA_CBC: begin
            case ({dc_exp_3, int_set})
                2'b10: st_next = START_PRE;
                2'b00: st_next = LOAD_DATA;
                2'b01: st_next = GET_DATA_CBC;
                2'b11: st_next = GET_DATA_CBC;
                default: st_next = SENT_DATA_CBC;
            endcase
        end
        LOAD_DATA: begin
            case (dc_exp_5)
                1'b1: st_next = LOAD_DATA; 
                default: st_next = START_PRE;
            endcase
        end
        GET_DATA_RBR: begin
            case (store_ddr_en)
                1'b0: st_next = START_PRE;
                1'b1: st_next = STORE_DATA;
                default: st_next = GET_DATA_RBR;
            endcase
        end
        GET_DATA_CBC: begin
            case (store_ddr_en)
                1'b0: st_next = START_PRE;
                1'b1: st_next = STORE_DATA;
                default: st_next = GET_DATA_CBC;
            endcase
        end
        STORE_DATA: begin
            case (dc_exp_7)
                1'b1: st_next = STORE_DATA; 
                default: st_next = STORE_DATA_END;
            endcase
        end
        STORE_DATA_END: begin
            st_next = START_PRE;
        end
        default: begin
            st_next = START_PRE;
        end
    endcase
end

endmodule
