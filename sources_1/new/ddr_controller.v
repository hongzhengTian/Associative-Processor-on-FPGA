module ddr_controller
#(
	parameter DDR_DATA_WIDTH = 128,
	parameter DDR_ADDR_WIDTH = 28
)
(
	input                           rst,                              
	input                           clk,     
	input                           rd_burst_req,                       
	input                           wr_burst_req,                       
	input [9:0]                     rd_burst_len,                  
	input [9:0]                     wr_burst_len,                   
	input [DDR_ADDR_WIDTH - 1:0]    rd_burst_addr,        
	input [DDR_ADDR_WIDTH - 1:0]    wr_burst_addr,        
	output                          rd_burst_data_valid,   
	output reg                      rd_burst_data_valid_delay,          
	output                          wr_burst_data_req,                    
	output [DDR_DATA_WIDTH - 1:0]   rd_burst_data,   
	input [DDR_DATA_WIDTH - 1:0]    wr_burst_data,    
	output                          rd_burst_finish,                     
	output                          wr_burst_finish, 
	input  							ddr_init_input_finish, 
	input [9 : 0]					wr_data_cnt_2,
	output                          burst_finish,  
	output reg [9:0]                rd_addr_cnt,                       
	
	///
   output [DDR_ADDR_WIDTH - 1 :0]   app_addr,
   output [2:0]                     app_cmd,
   output                           app_en,
   output [DDR_DATA_WIDTH-1:0]      app_wdf_data,
   output                           app_wdf_end,
   output [DDR_DATA_WIDTH/8-1:0]    app_wdf_mask,
   output                           app_wdf_wren,
   input [DDR_DATA_WIDTH-1:0]       app_rd_data,
   input                            app_rd_data_valid,
   input                            app_rdy,
   input                            app_wdf_rdy,
   input                            init_calib_complete
);
 
assign app_wdf_mask = {DDR_DATA_WIDTH/8{1'b0}};
 
localparam IDLE = 4'd0;
localparam MEM_READ = 4'd1;
localparam MEM_READ_WAIT = 4'd2;
localparam MEM_WRITE  = 4'd3;
localparam MEM_WRITE_2  = 4'd4;
localparam MEM_WRITE_WAIT = 4'd5;
localparam READ_END = 4'd6;
localparam WRITE_END = 4'd7;
localparam MEM_WRITE_FIRST_READ = 4'd8;

reg [3:0] state;	
reg [9:0] rd_data_cnt;
reg [9:0] wr_addr_cnt;
reg [9:0] wr_data_cnt;
reg [9:0] wr_data_cnt_2_delay;
reg [2:0] app_cmd_r;
reg [DDR_ADDR_WIDTH - 1:0] app_addr_r;
reg app_en_r;
reg app_wdf_wren_r;
reg [1 : 0]	wr_addr_add_cnt;

assign app_cmd = app_cmd_r;
assign app_addr = app_addr_r;
assign app_en = app_en_r;
assign app_wdf_end = app_wdf_wren;
assign app_wdf_data = wr_burst_data;
assign app_wdf_wren = app_wdf_wren_r & app_wdf_rdy;
assign rd_burst_finish = (state == READ_END);
assign wr_burst_finish = (state == WRITE_END);
assign burst_finish = rd_burst_finish | wr_burst_finish;
 
assign rd_burst_data = app_rd_data;
assign rd_burst_data_valid = app_rd_data_valid;
 
assign wr_burst_data_req = ((state == MEM_WRITE) || (state == MEM_WRITE_2)) & app_wdf_rdy ;

wire [9 : 0] arith_1 = rd_data_cnt + 1;
wire [9 : 0] arith_2 = wr_addr_cnt + 1;
wire [9 : 0] arith_3 = wr_data_cnt + 1;
wire [DDR_ADDR_WIDTH - 1:0] arith_4 = app_addr_r + 8;
wire [9 : 0] arith_5 = rd_addr_cnt + 1;

wire exp_1 = (rd_addr_cnt == rd_burst_len - 1)? 1 : 0;
wire exp_2 = (rd_data_cnt == rd_burst_len - 1)? 1 : 0;
wire exp_3 = (wr_addr_cnt == wr_burst_len - 1)? 1 : 0;
wire exp_4 = (wr_data_cnt == wr_burst_len - 1)? 1 : 0;
wire exp_5 = (app_wdf_rdy & init_calib_complete)? 1 : 0;
wire exp_6 = (~app_en_r & app_wdf_rdy)? 1 : 0;

always@(posedge clk or posedge rst) begin
	if (rst) begin
		app_wdf_wren_r <= 1'b0;
	end
	else if (exp_5) begin
		app_wdf_wren_r <= wr_burst_data_req;
	end
end

always @(posedge clk) begin
	rd_burst_data_valid_delay <= rd_burst_data_valid;
	wr_data_cnt_2_delay <= wr_data_cnt_2;
end

always @(posedge clk or posedge rst) begin
	if (rst) begin
		wr_addr_add_cnt <= 0;
	end
	else if (wr_addr_add_cnt == 2 && wr_burst_req) begin
		wr_addr_add_cnt <= 1;
	end
	else if (((state == MEM_WRITE) || (state == MEM_WRITE_WAIT)) && ddr_init_input_finish) begin
		wr_addr_add_cnt <= wr_addr_add_cnt + 1;
	end
	else begin
		wr_addr_add_cnt <= 0;
	end
end

always@(posedge clk or posedge rst) begin
	if (rst) begin
		state <= IDLE;
	end
	else if (init_calib_complete) begin
		case(state)
			IDLE: begin
				if (rd_burst_req) begin
					state <= MEM_READ;
				end
				else if (wr_burst_req) begin
					state <= MEM_WRITE;
				end
			end
			MEM_READ: begin
				if (app_rdy && exp_1) begin
					state <= MEM_READ_WAIT;
				end
				if (app_rd_data_valid && exp_2) begin
					state <= READ_END;
				end
			end
			MEM_READ_WAIT: begin
				if (app_rd_data_valid && exp_2) begin
					state <= READ_END;
				end
			end
			MEM_WRITE_FIRST_READ: begin
				state <= MEM_WRITE;
			end
			MEM_WRITE: begin
				if (wr_burst_data_req && exp_4 && (!ddr_init_input_finish)) begin	
					state <= MEM_WRITE_WAIT;
				end
				if (wr_burst_data_req && exp_4 && ddr_init_input_finish) begin	
					state <= MEM_WRITE_2;
				end
			end
			MEM_WRITE_2: begin
				state <= MEM_WRITE_WAIT;
			end
			READ_END: begin
				state <= IDLE;
			end
			MEM_WRITE_WAIT: begin
				if (app_rdy && exp_3 && app_wdf_rdy) begin
					state <= WRITE_END;
				end
				else if (exp_6) begin
					state <= WRITE_END;
				end
			end
			WRITE_END: begin
				state <= IDLE;
			end
			default: begin
				state <= IDLE;
			end
		endcase
	end
end

always@(posedge clk or posedge rst) begin
	if (rst) begin
		app_cmd_r <= 0;
		app_en_r <= 0;
		rd_addr_cnt <= 0;
		rd_data_cnt <= 0;
		app_addr_r <= 0;
	end
	else if (init_calib_complete) begin
		case(state)
			IDLE: begin
				if (rd_burst_req) begin
					app_cmd_r <= 3'b001;
					app_addr_r <= rd_burst_addr;
					app_en_r <= 1'b1;
				end
				else if (wr_burst_req) begin
					app_cmd_r <= 3'b000;
					app_addr_r <= wr_burst_addr;
					app_en_r <= 1'b1;
				end
			end
			MEM_READ: begin
				if (app_rdy) begin
					app_addr_r <= arith_4;
					if (exp_1) begin
						rd_addr_cnt <= 0;
						app_en_r <= 1'b0;
					end
					else begin
						rd_addr_cnt <= arith_5;
					end
				end
				if (app_rd_data_valid) begin
					if (exp_2) begin
						rd_data_cnt <= 0;
					end
					else begin
						rd_data_cnt <= arith_1;
					end
				end
			end
			MEM_READ_WAIT: begin
				if (app_rd_data_valid) begin
					if (exp_2) begin
						rd_data_cnt <= 0;
					end
					else begin
						rd_data_cnt <= arith_1;
					end
				end
			end
			MEM_WRITE_FIRST_READ: begin
				app_en_r <= 1'b1;
			end
			MEM_WRITE: begin
				if (app_rdy && (!ddr_init_input_finish)) begin
					app_addr_r <= arith_4;
					if (exp_3) begin
						app_en_r <= 1'b0;
					end
				end
				if (app_rdy && (ddr_init_input_finish)) begin
					if (wr_addr_add_cnt == 2) begin
						app_addr_r <= arith_4;
					end
					if (exp_3) begin
						app_en_r <= 1'b0;
					end
				end
			end
			MEM_WRITE_WAIT: begin
				if (app_rdy && (!ddr_init_input_finish)) begin
					app_addr_r <= arith_4;
					if (exp_3) begin
						app_en_r <= 1'b0;
					end
				end
				if (app_rdy && (ddr_init_input_finish)) begin
					if (wr_addr_add_cnt == 1) begin
						app_addr_r <= arith_4;
					end
					if (exp_3) begin
						app_en_r <= 1'b0;
					end
				end
			end
			default: ;
		endcase
	end
end

always@(posedge clk or posedge rst) begin
	if (rst) begin
		wr_addr_cnt <= 0;
	end
	else if (init_calib_complete) begin
		case(state)
			IDLE: begin
				if (wr_burst_req) begin
					wr_addr_cnt <= 0;
				end
			end
			MEM_WRITE_FIRST_READ: begin
				wr_addr_cnt <= 0;
			end
			MEM_WRITE: begin
				if (app_rdy && (!ddr_init_input_finish)) begin
					if (!exp_3) begin
						wr_addr_cnt <= arith_2;
					end
				end
				if (app_rdy && (ddr_init_input_finish)) begin
					if ((!exp_3) && (wr_addr_add_cnt == 2)) begin
						wr_addr_cnt <= arith_2;
					end
				end
			end
			MEM_WRITE_WAIT: begin
				if (app_rdy && (!ddr_init_input_finish)) begin
					if (!exp_3) begin
						wr_addr_cnt <= arith_2;
					end
				end
				if (app_rdy && (ddr_init_input_finish)) begin
					if ((!exp_3) && (wr_addr_add_cnt == 1)) begin
						wr_addr_cnt <= arith_2;
					end
				end
			end
			WRITE_END: begin
				wr_addr_cnt <= 0;
			end
			default: ;
		endcase
	end
end

always@(posedge clk or posedge rst) begin
	if (rst) begin
		wr_data_cnt <= 0;
	end
	else if (init_calib_complete) begin
		case(state)
			IDLE: begin
				if (wr_burst_req) begin
					wr_data_cnt <= 0;
				end
			end
			MEM_WRITE: begin
				if (wr_burst_data_req) begin
					if ((!exp_4) && (!ddr_init_input_finish)) begin
						wr_data_cnt <= arith_3;
					end
					if ((!exp_4) && (ddr_init_input_finish)) begin
						wr_data_cnt <= wr_data_cnt_2_delay;
					end
				end
			end
			WRITE_END: begin
				wr_data_cnt <= 0;
			end
			default: ;
		endcase
	end
end

endmodule