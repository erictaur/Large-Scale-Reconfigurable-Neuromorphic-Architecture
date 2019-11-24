`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    17:14:07 08/14/2019 
// Design Name: 
// Module Name:    core_16_top 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module core_16_top #(
	parameter data_width = 8
)
(
	input clk,
	input rst,
	// Data from USB (dfu) to FPGA
   input [31:0]  dfu_dout,
   input         dfu_empty,
   output reg    dfu_rd_en,
   input         dfu_valid,

   // Data to USB (from FPGA) FIFO
   input         dtu_full,
   output [7:0]  dtu_din,
   output reg    dtu_wr_en,
   input         dtu_wr_ack
    );

	//Packet
	reg  [5:0] coreID;
	reg  [7:0] command;
	reg  [1:0] idx_r;
	reg  [1:0] idx_c;
	reg  [7:0] value;
	
	//States
	reg  [3:0] curr_state, next_state, curr_state_2, next_state_2;
	
	//Flags										
	reg in_ram_din_wren;
	reg w_ram_din_wren;
	reg done_matmul;
	
	//Input_ram variables
	wire [1:0]  input_ram_addr;
	wire [31:0] input_ram_dout;
	wire [31:0] input_ram_din;
	reg  [3:0]  input_ram_wren;
	reg  [31:0] input_pad_data;
	wire [3:0]  decoded_row;
	reg  [7:0]   data_output;
	
	//Weight_ram variables
	wire [31:0] weight_ram_dout;
	wire [31:0] weight_ram_din;
	reg  [3:0]  weight_ram_wren;
	wire [3:0]  weight_ram_addr;
	reg  [31:0] weight_pad_data;
	
	//Result_ram variables
	wire [1:0]    result_ram_addr;
	wire [7:0]    result_ram_din;
	wire [7:0]    result_ram_dout;
	wire          result_ram_wren;
	
	//Matmul variables
	wire  [7:0]    data_0_sel;     
	wire  [7:0]    data_1_sel;
	wire  [7:0]    data_2_sel; 	
	wire  [7:0]    data_3_sel; 
	wire  [7:0]    weight_0_sel;  
	wire  [7:0]    weight_1_sel;
	wire  [7:0]    weight_2_sel; 
	wire  [7:0]    weight_3_sel; 
	
	reg  jump; // Flag
	reg  [7:0] acc_reg;
	wire [7:0] acc;
	reg  start;
	reg  [1:0] delay_counter_reg_1;
	reg  [1:0] delay_counter_reg_2;
	
	reg  reset_counters;
	reg  [3:0]   weight_counter;
	reg  [1:0]   row_counter;
	reg  [1:0]   output_counter;
	
	wire [7:0] row_dot_result;
	reg        delayed_reset;
	
	//instantiations
	input_RAM in_ram (
	  .clka   (clk),      // input clka
	  .wea    (input_ram_wren), // input  [3 : 0] wea
	  .addra  (input_ram_addr), // input  [1 : 0] addra
	  .dina   (input_ram_din),  // input  [31: 0] dina
	  .douta  (input_ram_dout)  // output [31: 0] douta
	);
	
	weight_RAM w_ram (
	  .clka   (clk),      // input clka
	  .wea    (weight_ram_wren), // input  [3 : 0]  wea
	  .addra  (weight_ram_addr), // input  [15 : 0] addra
	  .dina   (weight_ram_din),  // input  [31 : 0] dina
	  .douta  (weight_ram_dout)  // output [31 : 0] douta
	);
	
	out_RAM r_ram (
	  .clka   (clk),      // input clka
	  .wea    (result_ram_wren), // input  [0 : 0] wea
	  .addra  (result_ram_addr), // input  [3 : 0] addra
	  .dina   (result_ram_din),  // input  [7 : 0] dina
	  .douta  (result_ram_dout)  // output [7 : 0] douta
	);

	decoder2to4 decoder2to4(
		.a     (idx_c),
		.b     (decoded_row)
	);
	
	dot_float_top matmul (
		.x1(data_0_sel),
		.w1(weight_0_sel),
		.x2(data_1_sel),
		.w2(weight_1_sel),
		.x3(data_2_sel),
		.w3(weight_2_sel),
		.x4(data_3_sel),
		.w4(weight_3_sel),
		.row_dot_result(row_dot_result)
	);
	
	float_add acc_add (
	  .a(row_dot_result), // input [7 : 0] a
	  .b(acc_reg), // input [7 : 0] b
	  .result(acc) // output [7 : 0] result
	);
	
	//State Variables
	//FSM1
	parameter STATE_RESET             = 4'd0;
   parameter STATE_IDLE              = 4'd1;
   parameter STATE_GET_PACKET        = 4'd2;
   parameter STATE_VER_CMD           = 4'd3;
	parameter STATE_WRITE_IN_DATA_0   = 4'd4;
   parameter STATE_WRITE_IN_DATA_1   = 4'd5;
   parameter STATE_READ_IN_DATA      = 4'd6;
	parameter STATE_WRITE_W_DATA_0    = 4'd8;
	parameter STATE_WRITE_W_DATA_1    = 4'd9;
	parameter STATE_READ_W_DATA       = 4'd10;
	parameter STATE_READ_O_DATA       = 4'd11;
	parameter STATE_MATMUL            = 4'd12;
	
	//FSM2
	parameter STATE_FSM2_RESET             = 4'd0;
	parameter STATE_FSM2_PROCESS           = 4'd1;
	parameter STATE_FSM2_DONE              = 4'd2;
	
	//Assignments
	assign input_ram_din   = input_pad_data;        //Connect input data to din of input RAM
	//assign input_ram_addr  = idx_r;                 //Connect Row index to addr of input RAM
	assign weight_ram_din  = weight_pad_data;
	//assign weight_ram_addr = idx_r;    
	assign dtu_din         = data_output;	         //Connect Read data to input of outgoing FIFO	

	assign data_0_sel      = input_ram_dout [7:0];
	assign data_1_sel      = input_ram_dout [15:8];
	assign data_2_sel      = input_ram_dout [23:16];
	assign data_3_sel      = input_ram_dout [31:24];
	assign weight_0_sel    = weight_ram_dout[7:0];
	assign weight_1_sel    = weight_ram_dout[15:8];			
	assign weight_2_sel    = weight_ram_dout[23:16];
	assign weight_3_sel    = weight_ram_dout[31:24];
	
	
	// Wire output of last add operation straightly to the result RAM
	assign result_ram_din  = acc_reg;
	// Multiplexing wires to address ports of the RAMs
	assign result_ram_addr = (curr_state_2 == STATE_FSM2_PROCESS)? output_counter : idx_r;
	assign input_ram_addr  = (curr_state_2 == STATE_FSM2_PROCESS)? row_counter    : idx_r;
	assign weight_ram_addr = (curr_state_2 == STATE_FSM2_PROCESS)? weight_counter : idx_r;

	//FSM Driver
	always @(posedge rst or posedge clk) begin
      if (rst) begin
         curr_state    <= STATE_RESET;
			curr_state_2  <= STATE_FSM2_RESET;
      end else begin 
			curr_state_2  <= next_state_2;
			curr_state    <= next_state;
      end
   end


	//FSM 1 (Write/Read of input/weight RAMs)
	always @(*) begin
      
		start             = 0;
      dfu_rd_en         = 0;
      in_ram_din_wren   = 0;
		w_ram_din_wren    = 0;
      input_ram_wren    = 0;
		weight_ram_wren   = 0;
      dtu_wr_en         = 0;
		data_output       = 0;
      next_state        = curr_state;

      case(curr_state)
         STATE_RESET: begin
            next_state = STATE_IDLE;
         end
			
         STATE_IDLE: begin
				if(~dfu_empty)begin 
					dfu_rd_en  = 1;
					next_state = STATE_GET_PACKET;
				end
         end
			
			/*
			Packet segmentated at this state.
			Information of packet available at the next.
			*/
         STATE_GET_PACKET: begin
               next_state = STATE_VER_CMD;
         end
         
			STATE_VER_CMD: begin
				if (command == 8'd255) 
					next_state = STATE_WRITE_IN_DATA_0;
				else if (command == 8'd240)
					next_state = STATE_WRITE_W_DATA_0;
				else if (command == 8'd15)
					next_state = STATE_READ_IN_DATA;		
				else if (command == 8'd0)
					next_state = STATE_READ_W_DATA;
				else if (command == 8'd170)
					next_state = STATE_READ_O_DATA;
				else begin 
					start = 1;
					next_state = STATE_MATMUL;
				end

			end
			
         // Write Process (Input) starts here
         STATE_WRITE_IN_DATA_0: begin
				in_ram_din_wren = 1;
            next_state   = STATE_WRITE_IN_DATA_1;
         end

			STATE_WRITE_IN_DATA_1: begin 				         		
				input_ram_wren     = decoded_row;
				next_state         = STATE_IDLE;
			end
         
         // Write Process (Weight) starts here
         STATE_WRITE_W_DATA_0: begin
				w_ram_din_wren = 1;
            next_state     = STATE_WRITE_W_DATA_1;
         end
			
			STATE_WRITE_W_DATA_1: begin 				         		
				weight_ram_wren     = decoded_row;
				next_state          = STATE_IDLE;
			end			
         
			// Read Process (Input) starts here
         STATE_READ_IN_DATA: begin
            if (~dtu_full) begin
					data_output   = input_ram_dout[(idx_c + 1)*8-1 -: 8];
               dtu_wr_en     = 1;
               next_state    = STATE_IDLE;
            end
         end
			
			// Read Process (Weight) starts here
			STATE_READ_W_DATA: begin
            if (~dtu_full) begin
					data_output   = weight_ram_dout[(idx_c + 1)*8-1 -: 8];
               dtu_wr_en     = 1;
               next_state    = STATE_IDLE;
            end
         end
			
			STATE_READ_O_DATA: begin
            if (~dtu_full) begin
					data_output   = result_ram_dout;
               dtu_wr_en     = 1;
               next_state    = STATE_IDLE;
            end
         end
         
			/*
			Not sure how to control the flag signal in order to make sure this FSM 
			is halted while the other is running and vice versa.
			*/
			STATE_MATMUL: begin 
				if(done_matmul)begin
					next_state = STATE_IDLE;
				end
			end
			
         default: next_state = STATE_RESET;
      endcase
   end
	
	/*
	Segmentate packet data into respective component registers.
	*/
	always @(posedge clk) begin 
		if (curr_state == STATE_GET_PACKET)begin 
			coreID  <= dfu_dout[31:26];
			command <= dfu_dout[25:18];
			idx_r   <= dfu_dout[17:16];
			idx_c   <= dfu_dout[15:8];
			value   <= dfu_dout[7:0];	
		end
	end
	
	
	/*
	Write input/weight data to padded variables when flag is high.
	*/
	always @(posedge rst or posedge clk) begin
      if (rst) begin
         input_pad_data <= 32'd0;
			weight_pad_data <= 32'd0;
      end else if (in_ram_din_wren) begin 
			input_pad_data <= 32'd0;
			input_pad_data [(idx_c + 1)*data_width-1 -: 8] <= value;
		end else if (w_ram_din_wren) begin
			weight_pad_data <= 32'd0;
			weight_pad_data [(idx_c + 1)*data_width-1 -: 8] <= value;
		end
   end

	//FSM 2 (MATMUL operation)
	always @* begin 
		start_counters  = 0;
		reset_counters  = 0;
		done_matmul     = 0;
		next_state_2    = curr_state_2;
		
		case(curr_state_2) 
			
			STATE_FSM2_RESET: begin
				if(start)begin
					reset_counters = 1;
					next_state_2 = STATE_FSM2_PROCESS;
				end
         end

			STATE_FSM2_PROCESS: begin 
				
				start_counters = 1;
				
				//jump is the flag that signals the FSM to go to the done state
				if(jump == 1) begin 
					next_state_2 = STATE_FSM2_DONE;
				end

			end
			
			//Raises the flag to un-freeze FSM 1 
			STATE_FSM2_DONE: begin 
				done_matmul  = 1;
				next_state_2 = STATE_FSM2_RESET;
			end
			
			default: next_state_2 = STATE_FSM2_RESET;
			
		endcase
	end
	
	reg start_counters;
	
	//Weight RAM counter
	always @ (posedge clk) begin 
		if(reset_counters | weight_counter == 15)begin 
			weight_counter <= 0;
		end else if (start_counters) begin 
			weight_counter <= weight_counter + 1;
		end
	end
	
	always @ (posedge clk) begin 
		if (weight_counter == 15) begin 
			jump           <= 1;
		end else begin 
			jump           <= 0;
		end
	end
	
	always @ (posedge clk)begin 
		delayed_reset <= reset_counters;
	end
	
	//Input RAM counter
	always @ (posedge clk or posedge rst) begin 
		if(reset_counters | rst) begin 
			row_counter <= 0;
			delay_counter_reg_1 <=3;
			delay_counter_reg_2 <=2;
		end else if(row_counter == 3) begin 
			row_counter <= 0;
			delay_counter_reg_1 <= row_counter;
			delay_counter_reg_2 <= delay_counter_reg_1;
		end else if (start_counters) begin 
			row_counter <= row_counter + 1;
			delay_counter_reg_1 <= row_counter;
			delay_counter_reg_2 <= delay_counter_reg_1;
		end
	end
	
//	// Counter that has an offset of 2 compared to row counter
//	wire [1:0] delayed2_input_counter = {delay_counter_reg[1][0], delay_counter_reg[0][0]};
//	// Counter that has an offset of 1 compared to row counter
//	wire [1:0] delayed1_input_counter = {delay_counter_reg[1][1], delay_counter_reg[0][1]};
	// When delayed counter reaches 3, enable write enable of the result RAM
	assign result_ram_wren = (delay_counter_reg_1 == 3);
	
	always @ (posedge clk) begin 
		if(delayed_reset) begin // Reset after whole process 
			output_counter <= 0;
		end else if (delay_counter_reg_1 == 3) begin //Increment condition (dependent on delayed clock)
			output_counter <= output_counter + 1;
		end else begin 
			output_counter <= output_counter;
		end
	end

	
	//acc update
	always @ (posedge clk) begin 	
		if(reset_counters | delay_counter_reg_1 == 3)begin 
			acc_reg <= 0;
		end else begin 
			acc_reg <= acc; // acc_reg = acc + row_dot_result
		end 
	end

endmodule


// Decoder Module
module decoder2to4(a, b) ;
	parameter n=2 ;
	parameter m=4 ;

	input  [n-1:0] a ;
	output [m-1:0] b ;

	wire [m-1:0] b = 1<<a ;
endmodule

// MATMUL Module
module dot_float_top(
	input  [7:0] x1, w1, x2, w2, x3, w3, x4, w4,
	output [7:0] row_dot_result
    );

	wire [7:0] result1, result2, result3, result4, dot_result_0, dot_result_1;

	float_mult FM1 (
	  .a(x1), // input [7 : 0] a
	  .b(w1), // input [7 : 0] b
	  .result(result1) // output [7 : 0] result
	);
	
	float_mult FM2 (
	  .a(x2), // input [7 : 0] a
	  .b(w2), // input [7 : 0] b
	  .result(result2) // output [7 : 0] result
	);
	
	float_mult FM3 (
	  .a(x3), // input [7 : 0] a
	  .b(w3), // input [7 : 0] b
	  .result(result3) // output [7 : 0] result
	);
	
	float_mult FM4 (
	  .a(x4), // input [7 : 0] a
	  .b(w4), // input [7 : 0] b
	  .result(result4) // output [7 : 0] result
	);
	
	float_add AM1 (
	  .a(result1), // input [7 : 0] a
	  .b(result2), // input [7 : 0] b
	  .result(dot_result_0) // output [7 : 0] result
	);
	
	float_add AM2 (
	  .a(result3), // input [7 : 0] a
	  .b(result4), // input [7 : 0] b
	  .result(dot_result_1) // output [7 : 0] result
	);
	
	float_add AM3 (
	  .a(dot_result_0), // input [7 : 0] a
	  .b(dot_result_1), // input [7 : 0] b
	  .result(row_dot_result) // output [7 : 0] result
	);

endmodule
