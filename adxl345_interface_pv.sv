// ADXL345 Accelerometer Interface Physical Validation
// adxl345_interface_pv.sv
// Created By: Logan Beams
// Date: 05/08/2021

// 		----- Physical Validation of the ADXL345_Interface -----
//
// 	  This module is to validate the ADXL345_Interface works as intended
//	  on physical hardware. This in turn verifies that the instantiated 
//    modules for SPI communication work as well. The board used for physical
//    validation is the Intel/Altera DE10-Lite MAX 10 FPGA Development Board.
//	  The output data from the ADXL_Interface module is output to the onboard 
//    LEDs. The output data is 10 bits wide, so all 10 LEDs are used to output
//    the data in 2's complement binary format. The slide switches are used to
//    choose whether to display the x-, y-, or z-axis data. The KEY1 pushbutton
//    serves as the synchronous reset.
//	  SW1 HIGH = x-axis : SW2 HIGH = y-axis : SW3 HIGH = z-axis
//	  The undesired axis switches should be set low.
//    The default is to turn off the LEDs.
//

module ADXL345_Interface_pv (
	input				MAX10_CLK1_50,	// 50 MHz clock - PIN_P11
	input				KEY1,			// Push button 	- PIN_A7
	input				SW0,			// Switch		- PIN_C10
	input				SW1,			// Switch		- PIN_C11
	input				SW2,			// Switch		- PIN_D12
	input				SW3,			// Switch		- PIN_
	output logic [9:0] 	LEDR,			// LEDs (9 - 0) - PIN_B11, A11, D14, E14, C13, B10, A10, A9, A8
	output wire	[7:0]	HEX0,
	output wire [7:0]	HEX1,
	output wire [7:0]	HEX2,
	output wire [7:0]   HEX3,
	output wire			SCLK,			// Sensor SCLK	- PIN_AB15	
	output wire			CS_N,			// Sensor CS_N	- PIN_AB16
	inout wire			SDIO,			// Sensor SDIO	- PIN_V11
	input wire			INT1			// Sensor INT1	- PIN_Y14
	);
	
	logic 	[9:0]	r_x_data, r_y_data, r_z_data;	// Register to clock in the data.
	wire 	[9:0]	w_x_data, w_y_data, w_z_data;	// Output data from the ADXL345_Interface module
	wire	[8:0]	w_angle_x, w_angle_y;			// Angles of rotation
	wire 			w_data_valid;					// Data valid pulse from the ADXL345_Interface module
	
	ADXL345_Interface ADXL0 (MAX10_CLK1_50, KEY1, w_x_data, w_y_data, w_z_data, w_data_valid, SCLK, SDIO, CS_N, INT1);
	CORDIC   #(10)	  CORX	(MAX10_CLK1_50, KEY1, r_z_data, r_x_data, 1'b1, w_angle_x);
	CORDIC   #(10)	  CORY	(MAX10_CLK1_50, KEY1, r_z_data, r_y_data, 1'b1, w_angle_y);
	Format_7Seg		  F7S0	((SW3 ? w_angle_y : w_angle_x), HEX3, HEX2, HEX1, HEX0);
	
	// -- Combinatorial logic --
	// Output x-, y-, or z-axis to LEDs based on switch settings.
	always_comb begin
		case({SW2,SW1,SW0})
			3'b001 : LEDR = r_x_data;
			3'b010 : LEDR = r_y_data;
			3'b100 : LEDR = r_z_data;
			default: LEDR = '0;
		endcase
	end
	
	// -- Sequential logic --
	// Register the data when it is valid.
	always_ff @ (posedge MAX10_CLK1_50 or negedge KEY1)
		if (~KEY1) begin
			r_x_data <= '0;
			r_y_data <= '0;
			r_z_data <= '0;
		end
		else
			if (w_data_valid) begin
				r_x_data <= w_x_data;
				r_y_data <= w_y_data;
				r_z_data <= w_z_data;
			end

endmodule


// -- Format_7Seg Module --
// Converts a given 9-bit input into a 7 Segment Display format,
// consisting of a High Segment (100s), Mid Segment (10's), and Low Segment (1's).
// The expected input is the angle from the CORDIC modules, which should never exceed
// 180 degrees. The input data is in 2's complement, so a Sign Segment is also included.

module Format_7Seg (
	input [8:0] 		i_data, 
	output logic [7:0] 	o_SSeg,
	output logic [7:0] 	o_HSeg,
	output logic [7:0] 	o_MSeg,
	output logic [7:0] 	o_LSeg
	);
	
	logic [7:0]  w_data;			// Input data without the sign bit.
	logic [7:0]  w_data_min100;	// Input data minus 100.
	logic [7:0]	 w_data_10s;
	logic [7:0]  w_data_1s;
	
	assign w_data = i_data[8] ? -(i_data[7:0]) : i_data[7:0];  // If input is negative, take 2's complement.
	assign o_SSeg = i_data[8] ? 8'hBF : 8'hFF;		 		   // Turn on middle segment if negative input.
	
	Int_to_7Seg MS (w_data_10s, o_MSeg);
	Int_to_7Seg LS (w_data_1s, o_LSeg);
	
	always_comb begin
		if (w_data > 8'd99) begin 	// 100 or greater
			o_HSeg = 8'hF9;
			w_data_min100 = w_data - 8'd100;
		end  // if (w_data > 8'd99)
		else begin
			o_HSeg = 8'hC0;
			w_data_min100 = w_data;
		end  // else
	end  // always_comb
		
	always_comb begin
		w_data_10s = w_data_min100 / 8'd10;
		w_data_1s  = w_data_min100 - (w_data_10s * 8'd10);
	end  // always_comb
	
endmodule


module Int_to_7Seg (
	input 		 [7:0] i_int,
	output logic [7:0] o_seg
	);
	
	always_comb begin
		case (i_int)
			8'd1 : o_seg = 8'hF9;
			8'd2 : o_seg = 8'hA4;
			8'd3 : o_seg = 8'hB0;
			8'd4 : o_seg = 8'h99;
			8'd5 : o_seg = 8'h92;
			8'd6 : o_seg = 8'h82;
			8'd7 : o_seg = 8'hF8;
			8'd8 : o_seg = 8'h80;
			8'd9 : o_seg = 8'h98;
			default : o_seg = 8'hC0;
		endcase			
	end  // always_comb
	
endmodule







