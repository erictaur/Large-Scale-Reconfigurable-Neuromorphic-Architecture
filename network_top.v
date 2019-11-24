`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    22:49:24 08/17/2019 
// Design Name: 
// Module Name:    network_top 
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
module network_top(
   input sys_clk_p,
   input sys_clk_n,

   input sys_rst,


   // USB interface

	output CTS,			// clear to send (active low): FPGA ready to receive
	input  RTS,			// USB Clear to send
	output TX, 			// Output to USB
	input  RX, 			// Input to USB

   
   // Output LEDs
   output reg [3:0] leds
    );
	
	wire sysrst = sys_rst;


   // Clock signals
   wire clk200M;
   

   // USB controller 
   wire [31:0] dfu_dout;
   wire dfu_empty, dfu_rd_en, dfu_valid;
   
   wire [7:0] dtu_din;
   wire dtu_full, dtu_wr_en, dtu_wr_ack;
	
	clock_source clock_source_inst

   (// Clock in ports

    .CLK_IN1_P(sys_clk_p),    // IN

    .CLK_IN1_N(sys_clk_n),    // IN

    // Clock out ports

    .CLK_OUT1(clk200M));      // OUT

    
   assign sysclk = clk200M;

	core_16_top core(
	   .clk        (sysclk),
      .rst        (sysrst),
      // Data from USB (dfu) to FPGA
      .dfu_empty  (dfu_empty),
      .dfu_rd_en  (dfu_rd_en),
      .dfu_dout   (dfu_dout),
      .dfu_valid  (dfu_valid),
		// Data to USB (from FPGA) FIFO
      .dtu_full   (dtu_full),
      .dtu_din    (dtu_din),
      .dtu_wr_en  (dtu_wr_en),
      .dtu_wr_ack (dtu_wr_ack)
	);
	
	
	UART_IP uart_ip(
		.sysclk     (sysclk),

      .sysrst     (sysrst),

      

      .CTS        (CTS),
		.RTS        (RTS),
		.TX         (TX),
		.RX         (RX),


		.dfu_dout   (dfu_dout),
		.dfu_empty  (dfu_empty),
		.dfu_rd_en  (dfu_rd_en),
		.dfu_valid  (dfu_valid),

      .dtu_full   (dtu_full),
		.dtu_din    (dtu_din),
		.dtu_wr_en  (dtu_wr_en),

      .dtu_wr_ack (dtu_wr_ack)
	);

	always @(posedge sysclk) begin
      leds[2] <= dfu_empty;

      if (dtu_wr_en)
         leds[1:0] <= dtu_din[1:0];
   end


   // for debugging, leds[3] will blink at 1 Hz
   reg [26:0] counter;
   always @(posedge sysclk) begin

      if(counter < 100000000)
         counter <= counter + 1;
      else begin
         leds[3] <= ~leds[3];
         counter <= 0;
      end
   end
	
endmodule
   
   