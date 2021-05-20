// CORDIC Algorithim in Vectoring Mode
// cordic_vec.sv
// Created By: Logan Beams
// Date: 05/14/2021

// 		----- CORDIC Algorithim in Vectoring Mode -----
//
// 	  Converts a Cartesian vector into polar coordinates.
//    Only the resulting angle is desired (arctangent(y/x)),
// 	  so the inherent gain of the algorithim is ignored.
//	  The input x and y values should be in 2's complement form.
//    The resulting angle is in a fixed point 2's complement form.
//    The angle is 16 bits wide with a decimal point between the 8th/7th bits.
//	  Thus, the integer value is 8 bits wide with a sign bit, and the fractional value
//    is 7 bits wide. The LSB precision is 1/128 = 0.0078125.
//    Only the integer value of the angle is output.
//
module cordic_vec
  #(parameter N = 10) (
  input              i_clk,      // Input clock
  input              i_rst_n,    // Synchronous reset - active low
  input [N-1:0]      i_xval,     // Input x data - atan denominator
  input [N-1:0]      i_yval,     // Input y data - atan numerator
  input              i_en,       // Enable signal
  output logic [8:0] o_angle_int // Output angle integer value in degrees
  );

  // -- Local Parameters --
  localparam NUMITER    = 13; // Number of iterations	
  localparam PRECISION  = 9;  // Expand input bitwidth to improve precision	
  localparam ANGLEWIDTH = 16; // Bit-width of the angles

  // -- Local variables --
  logic signed [N-1+PRECISION:0] xval_array [NUMITER-1:0];   // Iteration array for the x-values
  logic signed [N-1+PRECISION:0] yval_array [NUMITER-1:0];   // Iteration array for the y-values
  logic signed [N-1+PRECISION:0] xval_expanded;              // Expanded x input to improve shift accuracy
  logic signed [N-1+PRECISION:0] yval_expanded;              // Expanded x input to improve shift accuracy
  logic        [ANGLEWIDTH-1:0]  angle_array  [NUMITER-1:0]; // Iteration array for angle results
  logic        [ANGLEWIDTH-1:0]  cordic_angle [NUMITER-1:0]; // Precalcuated CORDIC angles
  logic        [ANGLEWIDTH-1:0]  angle_final;                // Cumulative angle in degrees

  // -- Continuous assignments --
  assign angle_final   = angle_array[NUMITER-1]; // Output angle is the last iteration (cumulative) result
  assign xval_expanded = {i_xval,{PRECISION{1'b0}}}; // Append zeroes to mitigate rounding issues
  assign yval_expanded = {i_yval,{PRECISION{1'b0}}}; // Append zeroes to mitigate rounding issues
  assign o_angle_int   = (angle_final[6]) ? 
                         (angle_final[ANGLEWIDTH-1:7]+9'd1) :
                         (angle_final[ANGLEWIDTH-1:7]);
                         // angle_final[6] is the 1/2 bit. If high, round up. Otherwise, round down.

  // -- Sequential logic --
  // - Initial rotation based on the quadrant of the input (signs) -
  // The goal is to have a final angle that is positive if the input vector is in quadrant I or II
  // and negative angle is the input vector is in quadrant III or IV. This would be used to indicate
  // that a device has rotated counter clockwise (positive angle) or clockwise (negative angle).
  // Thus, if the vector is in quadrant I or II, it is rotated CW to be within 45 degrees of the x-axis (quadrant I).
  // If the vector is in quadrant III or IV, it is rotated CCW to be within -45 degrees of the x-axis (quadrant IV).
  always_ff @ (posedge i_clk) begin : InitialRotation
    if (~i_rst_n) begin
      xval_array[0]  <= '0;
      yval_array[0]  <= '0;
      angle_array[0] <= '0;
    end // if (~i_rst_n)
    else if (i_en) begin
      unique case ({i_xval[N-1],i_yval[N-1]}) // Evaluate the sign bit of each input.
        2'b00: begin // Quadrant I - CW Rotation
          xval_array[0]  <=  xval_expanded + yval_expanded;
          yval_array[0]  <= -xval_expanded + yval_expanded;
          angle_array[0] <= 16'b000101101_0000000; // 45 degrees CW
        end
        2'b10 : begin // Quadrant II - CW Rotation
          xval_array[0]  <= -xval_expanded + yval_expanded;
          yval_array[0]  <= -xval_expanded - yval_expanded;
          angle_array[0] <= 16'b010000111_0000000; // 135 degrees CW
        end
        2'b11 : begin // Quadrant III - CCW Rotation
          xval_array[0]  <= -xval_expanded - yval_expanded;
          yval_array[0]  <=  xval_expanded - yval_expanded;
          angle_array[0] <= -16'b010000111_0000000; // 135 degrees CCW
        end
        2'b01 : begin // Quadrant IV - CCW Rotation
          xval_array[0]  <=  xval_expanded - yval_expanded;
          yval_array[0]  <=  xval_expanded + yval_expanded;
          angle_array[0] <= -16'b000101101_0000000; // 45 degrees CCW
        end
      endcase
    end // else if (i_en)
  end : InitialRotation

  // - Rotation iterations -
  // Iteratively rotate the vector towards the x-axis by a CORDIC angle.
  // For each iteration, if the y-value is negative rotate CCW by the ith CORDIC angle. Otherwise CW.
  // Accumulate the angle during each iteration. Sum the CORDIC angle for CW rotation. Subtract for CCW.
  genvar i;
  generate
    for (i=0;i<NUMITER-1;i=i+1) begin : CordicIteration
      always_ff @ (posedge i_clk) begin
        if (~i_rst_n) begin
          xval_array[i+1]  <= '0;
          yval_array[i+1]  <= '0;
          angle_array[i+1] <= '0;
        end
        else if (i_en) begin
          if (yval_array[i] < 0) begin // y value is negative - rotate CCW
            xval_array[i+1]  <= xval_array[i] - (yval_array[i]>>>(i+1));
            yval_array[i+1]  <= yval_array[i] + (xval_array[i]>>>(i+1));
            angle_array[i+1] <= angle_array[i] - cordic_angle[i]; // Subtract the CORDIC angle
          end
          else begin                   // y value is positive - rotate CW
            xval_array[i+1]  <= xval_array[i] + (yval_array[i]>>>(i+1));
            yval_array[i+1]  <= yval_array[i] - (xval_array[i]>>>(i+1));
            angle_array[i+1] <= angle_array[i] + cordic_angle[i]; // Add the CORDIC angle
          end				
        end		
      end
    end : CordicIteration
  endgenerate

  // -- CORDIC Angles --
  // Represented as fixed point numbers, with the binary point located
  // between the 8th / 7th bit, annotated by the '_'. The MSB is a sign bit (2's complement)
  // LSB Precision = 1/128 in decimal
  assign cordic_angle[0]  = 16'b000011010_1001000; // 26.56500 degrees
  assign cordic_angle[1]  = 16'b000001110_0000101; // 14.03600 degrees
  assign cordic_angle[2]  = 16'b000000111_0010000; //  7.12500 degrees
  assign cordic_angle[3]  = 16'b000000011_1001010; //  3.57630 degrees
  assign cordic_angle[4]  = 16'b000000001_1100101; //  1.78990 degrees
  assign cordic_angle[5]  = 16'b000000000_1110011; //  0.89517 degrees
  assign cordic_angle[6]  = 16'b000000000_0111001; //  0.44761 degrees
  assign cordic_angle[7]  = 16'b000000000_0011101; //  0.22381 degrees
  assign cordic_angle[8]  = 16'b000000000_0001110; //  0.11191 degrees
  assign cordic_angle[9]  = 16'b000000000_0000111; //  0.05595 degrees
  assign cordic_angle[10] = 16'b000000000_0000100; //  0.02798 degrees
  assign cordic_angle[11] = 16'b000000000_0000010; //  0.01399 degrees
  assign cordic_angle[12] = 16'b000000000_0000001; //  0.00699 degrees
	
endmodule

/*
// Test Bench
module cordic_tb();
  logic clk, rst_n, en;
  logic [9:0] xval, yval;
  logic [8:0] angle_i;
  cordic_vec dut (clk, rst_n, xval, yval, en, angle_i);

  initial begin
    en='0; xval=10'd2; yval=10'd1;
    rst_n='0; #10; rst_n='1; #10;
    en='1;
    repeat (15) @(posedge clk);
    $display("x=%d, y=%d, atan(y/x)=%d degrees", $signed(xval), $signed(yval), $signed(angle_i));
    xval=10'd0; yval=-10'd2;
    repeat (15) @(posedge clk);
    $display("x=%d, y=%d, atan(y/x)=%d degrees", $signed(xval), $signed(yval), $signed(angle_i));
    xval=-10'd1; yval=10'd0;
    repeat (15) @(posedge clk);
    $display("x=%d, y=%d, atan(y/x)=%d degrees", $signed(xval), $signed(yval), $signed(angle_i));
    xval=-10'd1; yval=-10'd2;
    repeat (15) @(posedge clk);
    $display("x=%d, y=%d, atan(y/x)=%d degrees", $signed(xval), $signed(yval), $signed(angle_i));
  end

  initial begin
    clk='0; #5;
    repeat(100)
      repeat(2) begin clk=~clk; #5; end
  end

endmodule
*/