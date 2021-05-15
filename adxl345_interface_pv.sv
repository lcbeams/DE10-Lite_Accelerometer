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
//    The default is to display the x-axis data.
//

module ADXL345_Interface_pv (
	input				MAX10_CLK1_50,	// 50 MHz clock - PIN_P11
	input				KEY1,			// Push button 	- PIN_A7
	input				SW0,			// Switch		- PIN_C10
	input				SW1,			// Switch		- PIN_C11
	input				SW2,			// Switch		- PIN_D12
	output logic [9:0] 	LEDR,			// LEDs (9 - 0) - PIN_B11, A11, D14, E14, C13, B10, A10, A9, A8
	output wire			SCLK,			// Sensor SCLK	- PIN_AB15	
	output wire			CS_N,			// Sensor CS_N	- PIN_AB16
	inout wire			SDIO,			// Sensor SDIO	- PIN_V11
	input wire			INT1			// Sensor INT1	- PIN_Y14
	);
	
	logic 	[9:0]	r_x_data, r_y_data, r_z_data;	// Register to clock in the data.
	wire 	[9:0]	w_x_data, w_y_data, w_z_data;	// Output data from the ADXL345_Interface module
	wire 			w_data_valid;					// Data valid pulse from the ADXL345_Interface module
	
	ADXL345_Interface ADXL0 (MAX10_CLK1_50, KEY1, w_x_data, w_y_data, w_z_data, w_data_valid, SCLK, SDIO, CS_N, INT1);
	
	// -- Combinatorial logic --
	// Output x-, y-, or z-axis to LEDs based on switch settings.
	always_comb begin
		LEDR = r_x_data;
		case({SW2,SW1,SW0})
			3'b001 : LEDR = r_x_data;
			3'b010 : LEDR = r_y_data;
			3'b100 : LEDR = r_z_data;
			default: LEDR = r_x_data;
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