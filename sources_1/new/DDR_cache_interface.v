module DDR_cache_interface
#(
    parameter DDR_DATA_WIDTH    = 128,
	parameter DDR_ADDR_WIDTH    = 28,
	parameter ADDR_WIDTH_MEM	= 16,
	parameter DATA_WIDTH        = 16,
	parameter ISA_WIDTH			= 30,
    parameter ISA_DEPTH         = 72,
    parameter DATA_CACHE_DEPTH  = 16,
	parameter TOTAL_ISA_DEPTH   = 128,
	parameter TOTAL_DATA_DEPTH  = 64,
	parameter INT_INS_DEPTH 	= 27
)
(
	/*interface of system */
	input wire 								rst,                    /* reset*/
	input wire 								mem_clk,                /* interface clock*/
	input wire [ISA_WIDTH - 1 : 0]			ins_input,
	input wire [DATA_WIDTH - 1 : 0]			data_input,
	output wire                             load_ins_ddr,
	output wire                             load_data_ddr,
	output wire                             load_int_ins_ddr,

    /* interface of ISA_cache */
    input wire                              ins_read_req,
	input wire [DDR_ADDR_WIDTH - 1 : 0]		ins_read_addr,
    output reg [ISA_WIDTH - 1 : 0]			ins_to_cache,
	output reg [7 : 0]						rd_cnt_ins,
	output reg                              wr_en_ddr_to_ic_fifo,
	output reg                              ins_reading, // handshake signal with ins cache, reset ins_read_req
	input wire 								ddr_to_ic_fifo_empty,
	input wire [7 : 0]						ins_read_len,

    /* interface of DATA_cache */
    input wire                              data_read_req,
    input wire                              data_store_req,
	input wire								jmp_addr_read_req,
    input wire [DATA_WIDTH - 1 : 0]         data_to_ddr,
	input wire [9 : 0]						wr_data_cnt_1,
	input wire [DDR_ADDR_WIDTH - 1 : 0]		data_read_addr,
	input wire [DDR_ADDR_WIDTH - 1 : 0]		data_write_addr,
    output reg [DATA_WIDTH - 1 : 0]			data_to_cache,
	output reg [7 : 0] 						rd_cnt_data,
	output reg [DDR_ADDR_WIDTH - 1 : 0]		jmp_addr_to_cache,
	input wire                              ddr_to_dc_fifo_empty,
	output reg                              wr_en_ddr_to_dc_fifo,
	output reg                              data_reading,

	/* interface of ddr_controller */
	output reg                              ddr_init_input_finish,
	output reg [9 : 0] 					    wr_data_cnt_2,
	output reg 								rd_burst_req,           /* read request*/
	output reg 								wr_burst_req,           /* write request*/
	output reg [9 : 0] 						rd_burst_len,           /* read burst data length*/
	output reg [9 : 0] 						wr_burst_len,           /* write burst data length*/
	output reg [DDR_ADDR_WIDTH - 1 : 0] 	rd_burst_addr,        	/* starting addr of read burst*/
	output reg [DDR_ADDR_WIDTH - 1 : 0] 	wr_burst_addr,        	/* starting addr of write burst*/
	input wire 								rd_burst_data_valid,    /* data valid signal for readout*/
	input wire 								wr_burst_data_req,      /* ready for write, app_wdf_rdy*/
	(* DONT_TOUCH = "1" *)input wire [DDR_DATA_WIDTH - 1 : 0] 	rd_burst_data,   		/* readout data*/
	output reg [DDR_DATA_WIDTH - 1 : 0] 	wr_burst_data,    		/* write input data*/
	input wire 								rd_burst_finish,        /* read burst finish*/
	input wire 								wr_burst_finish        /* write burst finish*/
);

localparam START = 5'd0;               /*START=000; MEM_READ_ISA=001;MEM_WRITE=010;BURST_LEN_ISA=128*/
localparam MEM_WRITE_ISA = 5'd1;
localparam MEM_WRITE_ISA_END = 5'd2;
localparam MEM_WRITE_ISA_END_2 = 5'd3;
localparam MEM_WRITE_DATA = 5'd4;
localparam MEM_WRITE_DATA_END = 5'd5;
localparam MEM_WRITE_DATA_END_2 = 5'd6;
localparam MEM_WRITE_DATA_STORE = 5'd7;
localparam MEM_WRITE_DATA_STORE_END = 5'd8;
localparam MEM_WRITE_INT_ADDR = 5'd9;
localparam MEM_WRITE_INT_ADDR_END = 5'd10;
localparam MEM_WRITE_INT_ADDR_END_2 = 5'd11;
localparam MEM_WRITE_INT_INS = 5'd12;
localparam MEM_WRITE_INT_INS_END = 5'd13;
localparam MEM_WRITE_INT_INS_END_2 = 5'd14;
localparam MEM_READ_ISA = 5'd15;
localparam MEM_READ_ISA_END = 5'd16;
localparam MEM_READ_DATA = 5'd17;
localparam MEM_READ_DATA_END = 5'd18;
localparam MEM_READ_INT_ADDR = 5'd19;
localparam MEM_READ_INT_ADDR_END = 5'd20;

localparam W_ISA = 4'd1;
localparam W_DATA = 4'd2;
localparam R_ISA = 4'd3;
localparam R_DATA = 4'd4;
localparam W_DATA_STORE = 4'd5;
localparam W_INT_ADDR = 4'd6;
localparam R_INT_ADDR = 4'd7;
localparam W_INT_INS = 4'd8;

reg [3 : 0] CMD;
reg [4 : 0] state;
reg ddr_rdy;

wire finish_flag_w_isa;
wire finish_flag_w_data;
wire finish_flag_w_int_addr;
wire finish_flag_w_int_ins;

assign finish_flag_w_isa = ((state == MEM_WRITE_ISA_END) || (state == MEM_WRITE_ISA_END_2))? 1 : 0;
assign finish_flag_w_data = ((state == MEM_WRITE_DATA_END) || (state == MEM_WRITE_DATA_END_2))? 1 : 0;
assign finish_flag_w_int_addr = ((state == MEM_WRITE_INT_ADDR_END) || (state == MEM_WRITE_INT_ADDR_END_2))? 1 : 0;
assign finish_flag_w_int_ins = ((state == MEM_WRITE_INT_INS_END) || (state == MEM_WRITE_INT_INS_END_2))? 1 : 0;

assign load_ins_ddr = (state == MEM_WRITE_ISA);
assign load_data_ddr = (state == MEM_WRITE_DATA);
assign load_int_ins_ddr = (state == MEM_WRITE_INT_INS);

reg ddr_to_ic_fifo_empty_delay;
reg ddr_to_dc_fifo_empty_delay;

wire [9 : 0] arith_1;
wire [7 : 0] arith_2;
wire ddri_exp_2;
wire burst_finish;

assign arith_1 = rd_cnt_data + 1;
assign arith_2 = rd_cnt_ins + 1;
assign ddri_exp_2 = (state == MEM_WRITE_DATA_STORE)? 1 : 0;
assign burst_finish = rd_burst_finish || wr_burst_finish;

always @(posedge mem_clk or negedge rst) begin
	if (!rst) begin
		ddr_rdy <= 0;
	end
	else begin
		case (state)
			MEM_WRITE_INT_INS_END_2: ddr_rdy <= 1;
			default: ;
		endcase
	end
end

always @(posedge mem_clk) begin
	ddr_to_ic_fifo_empty_delay <= ddr_to_ic_fifo_empty;
	ddr_to_dc_fifo_empty_delay <= ddr_to_dc_fifo_empty;
	wr_data_cnt_2 <= wr_data_cnt_1;
end

/* WRITE part */
always@(*) begin 
    case (state)
        MEM_WRITE_ISA: wr_burst_data = {{(DDR_DATA_WIDTH - ISA_WIDTH){1'b0}},{ins_input}};
		MEM_WRITE_DATA: wr_burst_data = {{(DDR_DATA_WIDTH - DATA_WIDTH){1'b0}},{data_input}};
		MEM_WRITE_INT_ADDR: wr_burst_data = {{(DDR_DATA_WIDTH - 28){1'b0}}, 28'h0060000};  // interruption service program
		MEM_WRITE_INT_INS: wr_burst_data = {{(DDR_DATA_WIDTH - 28){1'b0}}, {ins_input}};
		MEM_WRITE_DATA_STORE: wr_burst_data = {{(DDR_DATA_WIDTH - DATA_WIDTH){1'b0}},{data_to_ddr}};
		default: wr_burst_data = 0;
    endcase
end

/* READ part */
always @ (posedge mem_clk or negedge rst) begin
	if (!rst) begin
		ins_to_cache <= 0;
		data_to_cache <= 0;
		jmp_addr_to_cache <= 0;
		ins_reading <= 0;
		data_reading <= 0;
	end
    else begin
        case (state)
            MEM_READ_ISA: begin
				ins_to_cache <= rd_burst_data[ISA_WIDTH - 1 : 0];
				ins_reading <= 1;
			end
            MEM_READ_DATA: begin
				data_to_cache <= rd_burst_data[DATA_WIDTH - 1 : 0];
				data_reading <= 1;
			end
            MEM_READ_INT_ADDR: begin
				jmp_addr_to_cache <= rd_burst_data[DDR_ADDR_WIDTH - 1 : 0];
				data_reading <= 1;
			end
            default: begin
				ins_reading <= 0;
				data_reading <= 0;
			end
        endcase
    end
end

/* cnt part */
always @(posedge mem_clk or negedge rst) begin
    if (!rst) begin
        rd_cnt_ins <= 0;
		rd_cnt_data <= 0;
		wr_en_ddr_to_ic_fifo <= 0;
		wr_en_ddr_to_dc_fifo <= 0;
    end
    else begin
        case ({state, rd_burst_data_valid})
            {MEM_READ_ISA,1'b1}: begin
				rd_cnt_ins <= arith_2;
				wr_en_ddr_to_ic_fifo <= 1;
			end
			{MEM_READ_ISA,1'b0}: ;
            {MEM_READ_DATA,1'b1}: begin
				rd_cnt_data <= arith_1;
				wr_en_ddr_to_dc_fifo <= 1;
			end
			{MEM_READ_DATA,1'b0}: ;
            {MEM_READ_INT_ADDR,1'b1}: begin
				rd_cnt_data <= arith_1;
				wr_en_ddr_to_dc_fifo <= 1;
			end
			{MEM_READ_INT_ADDR,1'b0}: ;
            default:begin
				rd_cnt_ins <= 0;
				rd_cnt_data <= 0;
				wr_en_ddr_to_ic_fifo <= 0;
				wr_en_ddr_to_dc_fifo <= 0;
			end
        endcase
    end
end

/* finish part */
always @(posedge mem_clk or negedge rst)
begin
	if(!rst)
	begin
		CMD <= W_ISA;
		wr_burst_len <= TOTAL_ISA_DEPTH;
		wr_burst_addr <= 0;
		wr_burst_req <= 1'b1;
		rd_burst_req <= 1'b0;
		rd_burst_addr <= 0;
		rd_burst_len <= 0;
		ddr_init_input_finish <= 0;
	end

	else if(burst_finish) begin
		rd_burst_req <= 0;
		wr_burst_req <= 0;
	end

	else begin
		case (ddr_rdy)
			1'b0: begin
				case ({finish_flag_w_isa, finish_flag_w_data, finish_flag_w_int_addr, finish_flag_w_int_ins})
					4'b1000: begin
						CMD <= W_DATA;
						wr_burst_len <= TOTAL_DATA_DEPTH + 1;
						wr_burst_addr <= 28'h0008000;
						wr_burst_req <= 1'b1;
						rd_burst_req <= 1'b0;
						rd_burst_addr <= 0;
						rd_burst_len <= 0;
					end
					4'b0100: begin
						CMD <= W_INT_ADDR;
						wr_burst_len <= 1 + 1;
						wr_burst_addr <= 28'h0070000;
						wr_burst_req <= 1'b1;
						rd_burst_req <= 1'b0;
						rd_burst_addr <= 0;
						rd_burst_len <= 0;
					end
					4'b0010: begin
						CMD <= W_INT_INS;
						wr_burst_len <= INT_INS_DEPTH + 1;
						wr_burst_addr <= 28'h0060000;
						wr_burst_req <= 1'b1;
						rd_burst_req <= 1'b0;
						rd_burst_addr <= 0;
						rd_burst_len <= 0;
					end
					4'b0001: begin
						ddr_init_input_finish <= 1;
					end
					default: CMD <= 0;
				endcase
			end
			1'b1: begin
				case ({data_read_req, jmp_addr_read_req, data_store_req, ins_read_req})
					4'b1000: begin
						CMD <= R_DATA;
						wr_burst_len <= 0;
						wr_burst_addr <= 0;
						wr_burst_req <= 1'b0;
						rd_burst_req <= 1'b1; 
						rd_burst_addr <= data_read_addr;
						rd_burst_len <= DATA_CACHE_DEPTH + 1;
					end 
					4'b0100: begin
						CMD <= R_INT_ADDR;
						wr_burst_len <= 0;
						wr_burst_addr <= 0;
						wr_burst_req <= 1'b0;
						rd_burst_req <= 1'b1; 
						rd_burst_addr <= data_read_addr;
						rd_burst_len <= 1;
					end
					4'b0010: begin
						CMD <= W_DATA_STORE;
						rd_burst_len <= 0;
						rd_burst_addr <= 0;
						wr_burst_req <= 1'b1;
						rd_burst_req <= 1'b0; 
						wr_burst_addr <= data_write_addr + 8;
						wr_burst_len <= DATA_CACHE_DEPTH;
					end
					4'b0001: begin
						CMD <= R_ISA;
						wr_burst_len <= 0;
						wr_burst_addr <= 0;
						wr_burst_req <= 1'b0;
						rd_burst_req <= 1'b1; 
						rd_burst_addr <= ins_read_addr;
						rd_burst_len <= ins_read_len;
					end
					default: CMD <= 0;
				endcase
			end
			default: ;
		endcase
	end
end

/* state machine part */
always@(posedge mem_clk or negedge rst) begin
	if(!rst) begin
		state <= START;
	end
	else begin
		case(state)
			START: begin
				case (CMD)
					W_ISA: state <= MEM_WRITE_ISA;
					W_DATA: state <= MEM_WRITE_DATA;
					R_ISA: begin
						if (ddr_to_ic_fifo_empty_delay) begin
							state <= MEM_READ_ISA;
						end
						else begin
							state <= START;
						end
					end
					R_DATA: begin
						if (ddr_to_dc_fifo_empty_delay) begin
							state <= MEM_READ_DATA;
						end
						else begin
							state <= START;
						end
					end
					W_INT_ADDR: state <= MEM_WRITE_INT_ADDR;
					W_INT_INS: state <= MEM_WRITE_INT_INS;
					R_INT_ADDR: begin
						if (ddr_to_dc_fifo_empty_delay) begin
							state <= MEM_READ_INT_ADDR;
						end
						else begin
							state <= START;
						end
					end
					W_DATA_STORE: state <= MEM_WRITE_DATA_STORE;
					default: state <= START;
				endcase
			end
			MEM_WRITE_ISA: begin
				if(wr_burst_finish) begin
					state <= MEM_WRITE_ISA_END;
				end
			end
			MEM_WRITE_ISA_END: begin
				state <= MEM_WRITE_ISA_END_2;
			end
			MEM_WRITE_ISA_END_2: begin
				state <= START;
			end
			MEM_WRITE_DATA: begin
				if(wr_burst_finish) begin
					state <= MEM_WRITE_DATA_END;
				end
			end
			MEM_WRITE_DATA_END: begin
				state <= MEM_WRITE_DATA_END_2;
			end
			MEM_WRITE_DATA_END_2: begin
				state <= START;
			end
			MEM_WRITE_INT_ADDR: begin
			  	if(wr_burst_finish) begin
				  	state <= MEM_WRITE_INT_ADDR_END;
				end
			end
			MEM_WRITE_INT_ADDR_END: begin
			  	state <= MEM_WRITE_INT_ADDR_END_2;
			end
			MEM_WRITE_INT_ADDR_END_2: begin
			  	state <= START;
			end
			MEM_WRITE_INT_INS: begin
				if(wr_burst_finish) begin
					state <= MEM_WRITE_INT_INS_END;
				end
			end
			MEM_WRITE_INT_INS_END: begin
			  	state <= MEM_WRITE_INT_INS_END_2;
			end
			MEM_WRITE_INT_INS_END_2: begin
			  	state <= START;
			end
			MEM_WRITE_DATA_STORE: begin
				if(wr_burst_finish) begin
					state <= MEM_WRITE_DATA_STORE_END;
				end
			end
			MEM_WRITE_DATA_STORE_END: begin
				state <= START;
			end
			MEM_READ_ISA: begin
				if(rd_burst_finish ) begin
					state <= MEM_READ_ISA_END;
				end
			end
			MEM_READ_ISA_END: begin
				state <= START;
			end
			MEM_READ_DATA: begin
			  	if(rd_burst_finish) begin
					state <= MEM_READ_DATA_END;
				end
			end
			MEM_READ_DATA_END: begin
				state <= START;
			end
			MEM_READ_INT_ADDR: begin
			  	if(rd_burst_finish) begin
					state <= MEM_READ_INT_ADDR_END;
				end
			end
			MEM_READ_INT_ADDR_END: begin
			  	state <= START;
			end
			default: state <= START;
		endcase
	end
end

endmodule