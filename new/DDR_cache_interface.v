module DDR_cache_interface
#(
    parameter DDR_DATA_WIDTH    = 128,
	parameter DDR_ADDR_WIDTH    = 28,
	parameter ADDR_WIDTH_MEM	= 16,
	parameter DATA_WIDTH        = 16,
	parameter ISA_WIDTH			= 30,
    parameter ISA_DEPTH         = 72,
    parameter DATA_CACHE_DEPTH  = 16,
	parameter TOTAL_ISA_DEPTH   = 64,
	parameter TOTAL_DATA_DEPTH  = 64
)
(
	/*interface of system */
	input wire 								rst,                    /* reset*/
	input wire 								mem_clk,                /* interface clock*/
	input wire [ISA_WIDTH - 1 : 0]			Instruction,
	input wire [DATA_WIDTH - 1 : 0]			Data,
	output reg [3 : 0] 						state,

    /* interface of ISA_cache */
    input wire                              ISA_read_req,
	input wire [DDR_ADDR_WIDTH - 1 : 0]		ISA_read_addr,
    output reg [ISA_WIDTH - 1 : 0]			instruction_to_cache,
	output reg         						ddr_rdy,
	output reg [9 : 0] 						rd_cnt_isa,
	input wire [9 : 0]						isa_read_len,

    /* interface of DATA_cache */
    input wire                              DATA_read_req,
    input wire                              DATA_store_req,
	input wire								JMP_ADDR_read_req,
    input wire [DATA_WIDTH - 1 : 0]         DATA_to_ddr,
	input wire                              data_to_ddr_rdy,
	input wire [DDR_ADDR_WIDTH - 1 : 0]		DATA_read_addr,
	input wire [DDR_ADDR_WIDTH - 1 : 0]		DATA_write_addr,
    output reg [DATA_WIDTH - 1 : 0]			DATA_to_cache,
	output reg [9 : 0] 						rd_cnt_data,
	output reg [DDR_ADDR_WIDTH - 1 : 0]		JMP_ADDR_to_cache,

	/* interface of ddr_controller */
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

localparam START 							= 4'd0;               /*START=000; MEM_READ_ISA=001;MEM_WRITE=010;BURST_LEN_ISA=128*/
localparam MEM_WRITE_ISA 					= 4'd1;
localparam MEM_WRITE_ISA_END 				= 4'd2;
localparam MEM_WRITE_DATA  					= 4'd3;
localparam MEM_WRITE_DATA_END 				= 4'd4;
localparam MEM_READ_ISA	 					= 4'd5;
localparam MEM_READ_ISA_END 				= 4'd6;
localparam MEM_READ_DATA 					= 4'd7;
localparam MEM_READ_DATA_END 				= 4'd8;
localparam MEM_WRITE_DATA_STORE 			= 4'd9;
localparam MEM_WRITE_DATA_STORE_END 		= 4'd10;
localparam MEM_WRITE_INT_ADDR 				= 4'd11;
localparam MEM_READ_INT_ADDR 				= 4'd12;
localparam MEM_WRITE_INT_ADDR_END		 	= 4'd13;
localparam MEM_READ_INT_ADDR_END 			= 4'd14;

localparam W_ISA 							= 3'd1;
localparam W_DATA 							= 3'd2;
localparam R_ISA 							= 3'd3;
localparam R_DATA 							= 3'd4;
localparam W_DATA_STORE 					= 3'd5;
localparam W_INT_ADDR						= 3'd6;
localparam R_INT_ADDR 						= 3'd7;

assign finish_flag_w_isa				 	= (state == MEM_WRITE_ISA_END);
assign finish_flag_w_data 					= (state == MEM_WRITE_DATA_END);
assign finish_flag_w_int_addr				= (state == MEM_WRITE_INT_ADDR_END);

assign finish_flag_r_isa 					= (state == MEM_READ_ISA_END);
assign finish_flag_r_data 					= (state == MEM_READ_DATA_END);
assign finish_flag_r_int_addr				= (state == MEM_READ_INT_ADDR_END);

reg [2 : 0] CMD;

/* WRITE part */
always@(posedge mem_clk or posedge rst)
begin
	if(rst)
	begin
		wr_burst_data   <= {DDR_DATA_WIDTH{1'b0}}; /* put 128 binary 0 to data register */
	end

	else if(state == MEM_WRITE_ISA)
	begin

		if(wr_burst_data_req)                       
			begin
				wr_burst_data   <= {{(DDR_DATA_WIDTH - ISA_WIDTH){1'b0}},{Instruction}};  
			end
	end

	else if(state == MEM_WRITE_DATA)
	begin
		if(wr_burst_data_req)                       
			begin
				wr_burst_data   <= {{(DDR_DATA_WIDTH - DATA_WIDTH){1'b0}},{Data}};  
			end
	end

	else if(state == MEM_WRITE_INT_ADDR)
	begin
		if(wr_burst_data_req)                       
			begin
				wr_burst_data   <= {{(DDR_DATA_WIDTH - 28){1'b0}}, 28'h00001a8};  
			end
	end

	else if (state == MEM_WRITE_DATA_STORE)
	begin
		if(wr_burst_data_req && data_to_ddr_rdy == 1)                       
			begin
				wr_burst_data   <= {{(DDR_DATA_WIDTH - DATA_WIDTH){1'b0}},{DATA_to_ddr}};  
			end
	end

	else wr_burst_data   <= 0;
end

/* READ part */
//always@(rd_burst_data_valid or rd_burst_data or state or rd_burst_finish)
always @ (posedge mem_clk or posedge rst)
begin
	if (rst)
		begin
			rd_cnt_isa 				<= 10'd0;
			rd_cnt_data 			<= 10'd0;
			instruction_to_cache 	<= 0;
			DATA_to_cache 			<= 0;
			JMP_ADDR_to_cache 		<= 0;
		end
	else if(state == MEM_READ_ISA)
		begin
			instruction_to_cache <= rd_burst_data[ISA_WIDTH - 1 : 0];
			if(rd_burst_data_valid)
				begin
					rd_cnt_isa <= rd_cnt_isa + 10'd1;
				end
			else if(rd_burst_finish)
			begin
				rd_cnt_isa <= 10'd0;
			end
				
		end

	else if(state == MEM_READ_DATA)
		begin
			DATA_to_cache <= rd_burst_data[DATA_WIDTH - 1 : 0];
			if(rd_burst_data_valid)
				begin
					rd_cnt_data <= rd_cnt_data + 10'd1;
				end
			else if(rd_burst_finish)
				rd_cnt_data <= 10'd0;
		end

	else if (state == MEM_READ_INT_ADDR)
		begin
			JMP_ADDR_to_cache <= rd_burst_data[DDR_ADDR_WIDTH - 1 : 0];
			if(rd_burst_data_valid)
				begin
					rd_cnt_data <= rd_cnt_data + 10'd1;
				end
			else if(rd_burst_finish)
				rd_cnt_data <= 10'd0;
		end

	else
	begin
		rd_cnt_isa <= 10'd0;
		rd_cnt_data <= 10'd0;
	end
end

/* finish part */
always @(posedge mem_clk or posedge rst or posedge finish_flag_w_isa or posedge finish_flag_w_data or posedge finish_flag_w_int_addr) 
begin
	if(rst)
	begin
		CMD <= W_ISA;
		ddr_rdy <= 0;
		wr_burst_req <= 1'b1;
		rd_burst_req <= 1'b0;
		rd_burst_addr <= 0;
		wr_burst_addr <= 0;
		
		wr_burst_len <= TOTAL_ISA_DEPTH;
		rd_burst_len <= 0;
	end	
	else if(finish_flag_w_isa)
	begin
	  	CMD <= W_DATA;
		wr_burst_len <= TOTAL_DATA_DEPTH + 1;
		wr_burst_addr <= 28'h0008000;
		wr_burst_req <= 1'b1;
		rd_burst_req <= 1'b0;
	end

	else if(finish_flag_w_data)
	begin
		CMD <= W_INT_ADDR;
		wr_burst_len <= 1 + 1; /* next we can change this by input the number */ //here we go//
		wr_burst_addr <= 28'h0070000;
		wr_burst_req <= 1'b1;
		rd_burst_req <= 1'b0;
	end

	else if(finish_flag_w_int_addr)
	begin
		ddr_rdy <= 1;
		CMD <= R_ISA;
		wr_burst_req <= 1'b0;
		rd_burst_req <= 1'b1; 
		rd_burst_addr <= ISA_read_addr;
		rd_burst_len <= isa_read_len;
	end

	else if(rd_burst_finish && (state == MEM_READ_DATA
								|| state == MEM_READ_ISA
								|| state == MEM_READ_INT_ADDR))
	begin
		rd_burst_req <= 0;
		wr_burst_req <= 0;
		CMD <= 0;
	end

	else if(state == MEM_WRITE_DATA_STORE)
	begin
		wr_burst_req <= DATA_store_req;
	end

	else if(ddr_rdy == 1 && DATA_read_req == 1)
	begin
		CMD <= R_DATA;
		wr_burst_req <= 1'b0;
		rd_burst_req <= 1'b1; 
		rd_burst_addr <= DATA_read_addr;
		rd_burst_len <= DATA_CACHE_DEPTH + 1;
	end

	else if(ddr_rdy == 1 && JMP_ADDR_read_req == 1)
	begin
		CMD <= R_INT_ADDR;
		wr_burst_req <= 1'b0;
		rd_burst_req <= 1'b1; 
		rd_burst_addr <= DATA_read_addr;
		rd_burst_len <= 1;
	end

	else if(ddr_rdy == 1 && DATA_store_req == 1)
	begin
		CMD <= W_DATA_STORE;
		wr_burst_req <= 1'b1;
		rd_burst_req <= 1'b0; 
		wr_burst_addr <= DATA_write_addr + 8;
		wr_burst_len <= DATA_CACHE_DEPTH; ///////
	end

	else  if(ddr_rdy == 1 && ISA_read_req == 1)
	begin
		CMD <= R_ISA;
		wr_burst_req <= 1'b0;
		rd_burst_req <= 1'b1; 
		rd_burst_addr <= ISA_read_addr;
		rd_burst_len <= isa_read_len;
	end

	else CMD <= 0;
end

/* state machine part */
always@(posedge mem_clk or posedge rst)
begin
	if(rst)
	begin
		state <= START;
		
	end
	else
	begin
		case(state)
			START:
			begin
				if(CMD == W_ISA)
				begin
					state <= MEM_WRITE_ISA;
					
				end

				else if(CMD == W_DATA)
				begin
					state <= MEM_WRITE_DATA;
					
				end

				else if(CMD == R_ISA)
				begin
					state <= MEM_READ_ISA;
					
				end

				else if(CMD == R_DATA)
				begin
					state <= MEM_READ_DATA;
				end

				else if(CMD == W_INT_ADDR)
				begin
				  	state <= MEM_WRITE_INT_ADDR;
				end

				else if(CMD == R_INT_ADDR)
				begin
				  	state <= MEM_READ_INT_ADDR;
				end

				else if(CMD == W_DATA_STORE)
				begin
					state <= MEM_WRITE_DATA_STORE;
				end

				else 
				begin
					state <= START;
				end
			end

			MEM_WRITE_ISA:
			begin
				if(wr_burst_finish)
				begin
					state <= MEM_WRITE_ISA_END;
				end
			end

			MEM_WRITE_ISA_END:
			begin
				state <= START;
			end

			MEM_WRITE_DATA:
			begin
				if(wr_burst_finish)
				begin
					state <= MEM_WRITE_DATA_END;
				end
			end

			MEM_WRITE_DATA_END:
			begin
				state <= START;
			end

			MEM_WRITE_INT_ADDR:
			begin
			  	if(wr_burst_finish)
				begin
				  	state <= MEM_WRITE_INT_ADDR_END;
				end
			end

			MEM_WRITE_INT_ADDR_END:
			begin
			  	state <= START;
			end

			MEM_WRITE_DATA_STORE:
			begin
				if(wr_burst_finish)
				begin
					state <= MEM_WRITE_DATA_STORE_END;
				end
			end

			MEM_WRITE_DATA_STORE_END:
			begin
				state <= START;
			end

			MEM_READ_ISA:
			begin
				if(rd_burst_finish )
				begin
					state <= MEM_READ_ISA_END;
				end
			end

			MEM_READ_ISA_END:
			begin
				state <= START;
			end

			MEM_READ_DATA:
			begin
			  	if(rd_burst_finish)
					begin
						state <= MEM_READ_DATA_END;
						
					end
			end

			MEM_READ_DATA_END:
			begin
				state <= START;
			end

			MEM_READ_INT_ADDR:
			begin
			  	if(rd_burst_finish)
					begin
						state <= MEM_READ_INT_ADDR_END;
					end
			end

			MEM_READ_INT_ADDR_END:
			begin
			  	state <= START;
			end

			default:
				state <= START;
		endcase
	end
end

endmodule