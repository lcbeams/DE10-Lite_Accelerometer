// ADXL345 Accelerometer Interface Physical Validation
// adxl345_interface_pv.sv
// Created By: Logan Beams
// Date: 05/08/2021

//         ----- Physical Validation of the ADXL345_Interface -----
//
//    This module is to validate that ADXL345 interface and submodules
//    works as intended on physical hardware. The board used for physical
//    validation is the Intel/Altera DE10-Lite MAX 10 FPGA Development Board.
//    The raw binary data output from the ADXL345 is displayed on the onboard
//    LEDs. The raw data is 10 bits wide, so all 10 LEDs are used to output
//    the data in 2's complement binary format. The angle of rotation in 
//    degrees is output to the onboard 7-Segment displays. The slide switches
//    are used to choose whether to display the x-, y-, or z-axis raw data,
//    and either the x- or y-axis rotation (Roll and Pitch)
//    The KEY1 pushbutton serves as the synchronous reset.
//    SW0 HIGH = x-axis : SW1 HIGH = y-axis : SW2 HIGH = z-axis raw data
//    The undesired axis switches should be set low.
//    The default is to turn off the LEDs.
//    SW3 LOW = x-axis rotation : SW3 HIGH = y-axis rotation
//

module adxl345_interface_pv (
  input              MAX10_CLK1_50, // 50 MHz clock - PIN_P11
  input              KEY1,          // Push button  - PIN_A7
  input              SW0,           // Switch       - PIN_C10
  input              SW1,           // Switch       - PIN_C11
  input              SW2,           // Switch       - PIN_D12
  input              SW3,           // Switch       - PIN_
  output logic [9:0] LEDR,          // LEDs (9-0)   - PIN_B11, A11, D14, E14, C13, B10, A10, A9, A8
  output logic [7:0] HEX0,
  output logic [7:0] HEX1,
  output logic [7:0] HEX2,
  output logic [7:0] HEX3,
  output logic       SCLK,          // Sensor SCLK  - PIN_AB15	
  output logic       CS_N,          // Sensor CS_N  - PIN_AB16
  inout wire         SDIO,          // Sensor SDIO  - PIN_V11
  input              INT1           // Sensor INT1  - PIN_Y14
  );

  logic [9:0] x_data, y_data, z_data;       // Register to clock store output data
  logic [9:0] x_data_o, y_data_o, z_data_o; // Output data from the ADXL345
  logic [8:0] angle_x, angle_y;             // Angles of rotation
  logic       data_valid;                   // Data valid pulse

  adxl345_interface adxl (MAX10_CLK1_50, KEY1, x_data_o, y_data_o, z_data_o, data_valid, SCLK, SDIO, CS_N, INT1);
  cordic_vec #(10)  corx (MAX10_CLK1_50, KEY1, z_data, x_data, 1'b1, angle_x);
  cordic_vec #(10)  cory (MAX10_CLK1_50, KEY1, z_data, y_data, 1'b1, angle_y);
  format_7seg       fseg ((SW3 ? angle_y : angle_x), HEX3, HEX2, HEX1, HEX0);

  // -- Combinatorial logic --
  // Output x-, y-, or z-axis to LEDs based on switch settings.
  always_comb begin
    case({SW2,SW1,SW0})
      3'b001 : LEDR = x_data;
      3'b010 : LEDR = y_data;
      3'b100 : LEDR = z_data;
      default: LEDR = '0;
    endcase
  end

  // -- Sequential logic --
  // Register the data when it is valid.
  always_ff @ (posedge MAX10_CLK1_50)
    if (~KEY1) begin
      x_data <= '0;
      y_data <= '0;
      z_data <= '0;
    end
    else if (data_valid) begin
      x_data <= x_data_o;
      y_data <= y_data_o;
      z_data <= z_data_o;
    end

endmodule

// -- format_7seg Module --
// Converts a given 9-bit input into a 7 Segment Display format,
// consisting of a High Segment (100s), Mid Segment (10's), and Low Segment (1's).
// The expected input is the angle from the CORDIC modules, which should never exceed
// 180 degrees. The input data is in 2's complement, so a Sign Segment is also included.

module format_7seg (
  input [8:0]        i_data, 
  output logic [7:0] o_SSeg,
  output logic [7:0] o_HSeg,
  output logic [7:0] o_MSeg,
  output logic [7:0] o_LSeg
  );

  logic [7:0] data_nosign; // Input data without the sign bit.
  logic [7:0] data_min100; // Input data minus 100.
  logic [7:0] data_10s;
  logic [7:0] data_1s;

  assign data_nosign = i_data[8] ? -(i_data[7:0]) : i_data[7:0]; // If input is negative, take 2's complement.
  assign o_SSeg = i_data[8] ? 8'hBF : 8'hFF;                     // Turn on middle segment if negative input.

  int_to_7seg MS (data_10s, o_MSeg);
  int_to_7seg LS (data_1s, o_LSeg);

  always_comb begin
    if (data_nosign > 8'd99) begin // 100 or greater
      o_HSeg = 8'hF9;
      data_min100 = data_nosign - 8'd100;
    end // if (w_data > 8'd99)
    else begin
      o_HSeg = 8'hC0;
      data_min100 = data_nosign;
    end // else
  end // always_comb
	
  always_comb begin
    data_10s = data_min100 / 8'd10;
    data_1s  = data_min100 - (data_10s * 8'd10);
  end // always_comb

endmodule

// int_to_7seg Module
// Converts an integer input into 7-Segment display format.
module int_to_7seg (
  input        [7:0] i_int,
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
  end // always_comb

endmodule







