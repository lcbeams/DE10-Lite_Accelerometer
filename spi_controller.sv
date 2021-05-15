// SPI Controller
// spi_controller.sv
// Created by: Logan Beams
// Date: 05/08/2021

// 					--- SPI Controller Module ---
//	   	This module implement a SPI main device to communicate with
//      a SPI secondary device. The intended secondary device is an
//      ADXL345 3-axis accelerometer. A 3-wire interface is used.
//
//		The SPI clock has CPOL = 1 and CPHA = 1. A Chip Select line is used
//      which idles high, and must be brought high inbetween each transmission.
//      Each transmission is 16 bits long, as required by the ADXL345.
//
//      The SPI clock is generated by a separate SPI_Clock_Generator module.
//      This modules uses clock edge pulses to enable the sequential logic
//      in the FPGA, which will receive the onboard 50 MHz clock. A divided down
//      clock in sync with these pulses is send to the secondary device.
//
//      The SPI communication begins when an spi_go signal is received, and ends
//      when the 16 bits have been sent/read, after which an idle signal is set.
//      When reading data from the ADXL345, a data_valid flag is set as well.
//
module SPI_Controller (
	// -- System inputs --
	input 				i_clk,			// System clock
	input 				i_rst_n,		// Asynchronous reset - active low				
	
	// -- Control signals --
	output logic		o_data_valid,  	// Signal valid data to be read
	input 				i_spi_go,		// Signal to begin data transfer
	input 				i_read_write_n,	// Signal to read or write data. 1 = Read, 0 = Write
	output wire			o_idle,			// Signal SPI controller is idling.
	
	// -- Input data from system --
	input [15:0] 		i_data,  		// Input data from the system to write to secondary device
	
	// -- Output data to system --
	output wire [7:0] 	o_data,			// Output data to the system from secondary device
	
	// -- Connections to secondary devices -- 
	output wire 		o_sclk,			// Output clock to secondary device
	inout wire	 		io_sdio,   		// Secondary device data input and output.
	output logic 		o_cs_n			// Chip select - active low. (Only one device selectable)
	);
	
	// -- Local parameters --
	localparam MAX_SCLK_COUNT  = 4'd15;	// Count sixteen positive clock edges for a full transfer
	localparam ONE			   = 4'd1;  // 4-bit one for decrementing the clock count
	
	// -- Variable declarations --
	logic [15:0] 	r_sdio_buffer;		// Data to be shifted in / shifted from secondary device
	logic [3:0]  	r_sclk_count;		// Count of positive sclk edges when sclk is enabled
	logic 			r_sclk_enable;		// Flag to enable the output sclk
	logic			r_sdio_enable;		// Flag to enable driving the sdio tristate output
	logic			r_spi_done;			// Pulse to indicate the spi controller is done
	logic			r_sdio;				// Register for setting up the next sdio output
	logic			r_read_write_n;		// Register for storing the input read/write request
	wire			w_riseEdge;			// riseEdge output from SPI_Clock_Generator - used by FPGA logic as pseudo sclk
	wire			w_fallEdge;			// fallEdge output from SPI_Clock_Generator - used by FPGA logic as pseudo sclk
	wire 			w_sclk;				// slowClk output from SPI_Clock_Generator - clock for secondary device
	
	// -- Continuous assignments --
	assign o_sclk  = (r_sclk_enable) ? w_sclk : '1;		// Drive sclk high until enabled
	assign io_sdio = (r_sdio_enable) ? r_sdio : 1'bz;  	// Drive the output with the r_sdio value when enabled. Otherwise, high-z.
	assign o_data  = r_sdio_buffer[7:0];				// Reading 8-bit from secondary device and storing in LSBs of buffer.
	assign o_idle  = o_cs_n;
	
	// -- Module Instantations --
	SPI_Clock_Generator SPICG0 (.i_clk(i_clk), .i_rst_n(i_rst_n), .o_riseEdge(w_riseEdge), .o_fallEdge(w_fallEdge), .o_slowClk(w_sclk));
	
	// -- Sequential logic --
	// This block starts the spi transfer process when a spi_go signal is received,
	// and stops the process when the spi transfer is done. This block intentionally
	// delays allowing the process to start again by one clock cycle, to ensure
	// that the chip select line is driven high for one clock cycle.
	always_ff @ (posedge i_clk or negedge i_rst_n)
		if (~i_rst_n) begin
			o_cs_n <= '1;
		end  // if (~i_rst_n)
		else begin
			if (r_spi_done) begin
				o_cs_n <= '1;						// Drive chip select line high while idle
			end  // (r_spi_done)
			else if (i_spi_go) begin
				o_cs_n <= '0;						// Pull chip select down to initiate transfer.
				r_read_write_n <= i_read_write_n;	// Store whether read or write request
			end  // if (i_spi_go)
		end  // else
			
	// This block sets the flag to enable the output sclk
	// The output sclk is only enabled after chip select is driven low
	// and only when the current state of the sclk is high, to avoid an 
	// erroneous clock edges. The output sclk is disabled when the transfer is complete.
	always_ff @ (posedge i_clk or negedge i_rst_n)
		if (~i_rst_n)
			r_sclk_enable <= '0;
		else
			if (~o_cs_n) begin 				// Chip select line is low
				if (w_sclk)					// Output sclk is positive
					r_sclk_enable <= '1;	// Enable output sclk
			end // if (~o_cs_n)
			else
				r_sclk_enable <= '0;		// Disable otherwise
			
	// This block triggers the spi_done pulse and o_data_valid pulse
	// The transfer is complete after the sclk count reaches zero.
	always_ff @ (posedge i_clk or negedge i_rst_n)
		if (~i_rst_n) begin
			r_spi_done <= '0;
			o_data_valid <= '0;
		end  // if (~i_rst_n)
		else begin
			if (~|r_sclk_count & w_riseEdge) begin		// Count is complete
				r_spi_done <= '1;						// SPI transfer is done
				if (r_read_write_n)
					o_data_valid <= '1;					// Data valid if reading data
			end  // if (~|r_sclk_count)
			else begin									// Otherwise zero
				r_spi_done <= '0;	
				o_data_valid <= '0;
			end  // else
		end  // else

	// This block counts the number of positive sclk edges when o_sclk is enabled
	// The count is reset back to the maximum count after the output sclk is disabled.
	// This counter counts down.
	always_ff @ (posedge i_clk or negedge i_rst_n)
		if (~i_rst_n)
			r_sclk_count <= MAX_SCLK_COUNT;
		else begin
			if (r_sclk_enable) begin						// Output clock enabled
				if (w_riseEdge)			 					// Count rising edges
					r_sclk_count <= r_sclk_count - ONE;		// Decrement count
			end  // if (r_sck_enable)
			else
				r_sclk_count <= MAX_SCLK_COUNT;				// Reset count when output clock is disabled
		end  // else
				
					
	// This block controls the transfer of data between the SPI Main and SPI Secondary device.
	// The input data is loaded into the spi buffer when the spi_go signal is received.
	// When in write mode, the sdio output is always driven by the sdio register and 16-bits are written.
	// The sdio register is set to the current count sdio buffer value on the falling edge of the sclk.
	// When in read mode, the sdio output is disabled after the eighth positive sclk edge.
	// Initially, 8-bits of data are written to the secondary device, then it is expected that the 
	// Secondary device will return 8-bits of data. 
	always_ff @ (posedge i_clk or negedge i_rst_n)
		if (~i_rst_n) begin
			r_sdio_enable <= '0;
		end  // if (~i_rst_n)
		else begin
			if (~o_cs_n) begin	
				if ((r_sclk_count > 7) | (~r_read_write_n)) begin	// Always execute if writing. If read, only writing 8 bits
					r_sdio_enable <= '1;							// Enable to sdio output
					if (w_fallEdge)
						r_sdio <= r_sdio_buffer[r_sclk_count];		// Setup the bit on the falling edge
				end  // if ((r_sclk_count > 7) | (~r_read_write_n))
				else begin 											// Read data request - Read 8-bits after writing
					r_sdio_enable <= '0;							// Disable the sdio output
					if (w_riseEdge)
						r_sdio_buffer[r_sclk_count] <= io_sdio;		// Register the bit on the rising edge
				end  // else
			end // if (~os_c_n)
			else begin
				r_sdio_enable <= '0;								// Disable the sdio output otherwise
				if (i_spi_go)
					r_sdio_buffer <= i_data;  						// Store input data if starting communication
			end  // else
		end  // else
		
endmodule


//					--- SPI Clock Generator Module ---
//
//	   	This module generates 'positive edge' and 'negative edge' signals
//	   	to be used as register enable signals to create a pseudo divided
//		down clock for logic within the FPGA. The module also outputs the
//		divided down clock for use by peripheral devices. The output clock
//		is delays by one input clock to align it with the pseudo clock.
//		The parameters are such that a 50 MHz onboard FPGA clock will 
//		a create 1 MHz output clock. This is intended for SPI communication.
//
module SPI_Clock_Generator (
	// -- System inputs --
	input 		 i_clk,				// FPGA clock - 50 MHz
	input 		 i_rst_n,			// Reset - Enable low
	
	// -- SPI Control Clocks --
	output logic o_riseEdge,		// Rising edge signal
	output logic o_fallEdge,   		// Falling edge signal
	output logic o_slowClk			// Divided down clock - 1 MHz
	);
	
	localparam MAX_COUNT = 5'd24;	// Maximum count to divide 50 MHz to 1 MHz
	localparam ONE = 5'd1;		    // 5-bit one for incrementing the count.
	
	logic [4:0] r_count;			// Hold the count.
	logic r_nextEdge;				// Signals the next edge. 0 = Falling, 1 = Rising
	logic r_slowClk;				// Divided down clock. Store in a register to implement a delay
	
	// Generate the riseEdge and fallEdge signals and the slow clock.
	always_ff @ (posedge i_clk or negedge i_rst_n)
		if (~i_rst_n) begin
			o_riseEdge <= '0;
			o_fallEdge <= '0;
			r_count <= '0;
			r_nextEdge <= '0;
			r_slowClk <= '1;
		end  // if (~i_rst_n)
		else begin
			r_count <= r_count + ONE;		// Increment count
			o_riseEdge <= '0;				// Reset enable signal
			o_fallEdge <= '0;				// Reset enable signal
			if (r_count == MAX_COUNT) begin
				if (r_nextEdge)
					o_riseEdge <= '1;		// Create the rising edge pulse
				else
					o_fallEdge <= '1;		// Create the falling edge pulse
				r_count <= '0;				// Reset count
				r_nextEdge <= ~r_nextEdge;	// Invert so the next edge is signaled
				r_slowClk <= ~r_slowClk;	// Invert the divided down clock - store in a register
			end  // if (count == MAX_COUNT)
		end  // else
		
	// Delay the slow clock by one input clock cycle.
	always_ff @ (posedge i_clk or negedge i_rst_n)
		if (~i_rst_n)
			o_slowClk <= '0;
		else
			o_slowClk <= r_slowClk;
	
endmodule

/*
// Test Bench
module SPI_Controller_TB();
	logic i_clk, i_rst_n, o_data_valid, i_spi_go, i_read_write_n, o_idle;
	logic [15:0] i_data;
	logic [7:0] o_data;
	logic o_sclk, o_cs_n;
	wire io_sdio;
	logic io_sdio_driver;
	logic [7:0] data_save;
	
	SPI_Controller DUT (i_clk, i_rst_n, o_data_valid, i_spi_go, i_read_write_n, o_idle, i_data, o_data, o_sclk, io_sdio, o_cs_n);
	assign io_sdio = io_sdio_driver;
	
	initial begin
		i_spi_go='0; i_read_write_n='0; i_data='0; io_sdio_driver=1'bz; data_save=8'b00000000;
		i_rst_n='0; #5; i_rst_n='1; #5;
		i_spi_go='1; i_read_write_n='0; i_data=16'b0101010101010101; #5;
		@(negedge o_idle) @(posedge i_clk) begin i_spi_go='0; i_read_write_n='0; end
		@(posedge o_idle) @(posedge i_clk) begin i_spi_go='1; i_read_write_n='1; i_data=16'b1010101000000000; end
		@(negedge o_idle) @(posedge i_clk) begin i_spi_go='0; i_read_write_n='0; end
		repeat(8) @(posedge o_sclk);
		@(negedge o_sclk) io_sdio_driver<='1;
		repeat (7) @(negedge o_sclk) io_sdio_driver<=~io_sdio_driver;
		@(posedge o_data_valid) @(posedge i_clk) data_save<=o_data;
		@(posedge o_idle) io_sdio_driver<=1'bz;
	end
	
	initial begin
		i_clk='0; #5;
		repeat(2000)
			repeat(2) begin i_clk=~i_clk; #5; end
	end
endmodule
*/

