// SPI Controller
// spi_controller.sv
// Created by: Logan Beams
// Date: 05/08/2021

//                  --- SPI Controller Module ---
//    This module implements an SPI main device to communicate with
//    an SPI secondary device. The intended secondary device is an
//    ADXL345 3-axis accelerometer. A 3-wire interface is used.
//
//    The SPI clock has CPOL = 1 and CPHA = 1. A Chip Select line is used
//    which idles high, and must be brought high between each transmission.
//    Each transmission is 16 bits long, as required by the ADXL345.
//
//    This module uses clock edge pulses to enable the sequential logic
//    in the FPGA, which will receive the onboard 50 MHz clock. A divided down
//    clock in sync with these pulses is send to the secondary device. This
//    is provided by a separate SPI Clock Generator module.
//
//    The SPI communication begins when an spi_go signal is received, and ends
//    when the 16 bits have been sent/read, after which an idle signal is set.
//    When reading data from the ADXL345, a data_valid flag is set as well.
//
module spi_controller (
  // -- System inputs --
  input              i_clk,          // System clock
  input              i_rst_n,        // Synchronous reset - active low				
  // -- Control signals --
  output logic       o_data_valid,   // Signal valid data to be read
  input              i_spi_go,       // Signal to begin data transfer
  input              i_read_write_n, // Signal to read or write data. 1 = Read, 0 = Write
  output logic       o_idle,         // Signal SPI controller is idling.
  // -- Input data from system --
  input [15:0]       i_data,         // Input data from the system to write to secondary device
  // -- Output data to system --
  output logic [7:0] o_data,         // Output data to the system from secondary device
  // -- Connections to secondary devices -- 
  output logic       o_sclk,         // Output clock to secondary device
  inout wire         io_sdio,        // Secondary device data input and output.
  output logic       o_cs_n          // Chip select - active low. (Only one device selectable)
	);

  // -- Local parameters --
  localparam MAX_SCLK_COUNT = 4'd15; // Count sixteen positive clock edges for a full transfer
  localparam ONE            = 4'd1;  // 4-bit one for decrementing the clock count

  // -- Variable declarations --
  logic [15:0] sdio_buffer;  // Data to be shifted in / shifted from secondary device
  logic [3:0]  sclk_count;   // Count of positive sclk edges when sclk is enabled
  logic        sclk_enable;  // Flag to enable the output sclk
  logic        sdio_enable;  // Flag to enable driving the sdio tristate output
  logic        spi_done;     // Pulse to indicate the spi controller is done
  logic        sdio;         // Register for setting up the next sdio output
  logic        read_write_n; // Register for storing the input read/write request
  logic        rise_edge;    // riseEdge output from SPI_Clock_Generator - used by FPGA logic as pseudo sclk
  logic        fall_edge;    // fallEdge output from SPI_Clock_Generator - used by FPGA logic as pseudo sclk
  logic        sclk;         // slowClk output from SPI_Clock_Generator - clock for secondary device

  // -- Continuous assignments --
  assign o_sclk  = (sclk_enable) ? sclk : '1;   // Drive sclk high until enabled
  assign io_sdio = (sdio_enable) ? sdio : 1'bz; // Drive the output with the r_sdio value when enabled. Otherwise, high-z.
  assign o_data  = sdio_buffer[7:0];            // Reading 8-bit from secondary device and storing in LSBs of buffer.
  assign o_idle  = o_cs_n;

  // -- Module Instantations --
  spi_clock_generator spicg (.i_clk, .i_rst_n, .o_rise_edge(rise_edge), .o_fall_edge(fall_edge), .o_clk_slow(sclk));

  // -- Sequential logic --
  // This block controls the chip select line.
  // Chip select idles high and drops low to begin the transmission process.
  // This block intentionally delays allowing the process to start again by 
  // one clock cycle, to ensure that the chip select line is driven high first.
  always_ff @ (posedge i_clk) begin : ChipSelect
    if (~i_rst_n) begin
      o_cs_n <= '1;
    end // if (~i_rst_n)
    else begin
      if (spi_done) begin
        o_cs_n <= '1;                   // Drive chip select line high while idle
      end // (spi_done)
      else if (i_spi_go) begin
        o_cs_n <= '0;                   // Pull chip select down to initiate transfer.
        read_write_n <= i_read_write_n; // Store whether read or write request
      end // if (i_spi_go)
    end // else
  end : ChipSelect

  // This block sets the flag to enable the output sclk.
  // The output sclk enabled after chip select is driven low, but
  // only when the current state of the sclk is high, to avoid a false edge.
  always_ff @ (posedge i_clk) begin : SclkEnable
    if (~o_cs_n) begin     // Chip select line is low
      if (sclk) begin      // Output sclk is positive
        sclk_enable <= '1; // Enable output sclk
      end // if (sclk)
    end // if (~o_cs_n)
	else begin
      sclk_enable <= '0;   // Disable otherwise
    end // else
  end : SclkEnable

  // This block triggers the spi_done pulse and o_data_valid pulse
  // The transfer is complete after the sclk count reaches zero.
  always_ff @ (posedge i_clk) begin : SpiDoneDataValid
    if (~|sclk_count & rise_edge) begin // Count is complete
      spi_done <= '1;                   // SPI transfer is done
      o_data_valid <= read_write_n;     // Data valid if reading data
    end // if (~|sclk_count & rise_edge)
    else begin                          // Otherwise zero
      spi_done <= '0;	
      o_data_valid <= '0;
    end // else
  end : SpiDoneDataValid

  // This block counts the number of positive sclk edges when o_sclk is enabled.
  // The count is reset back to the maximum count after the output sclk is disabled.
  always_ff @ (posedge i_clk) begin : SclkCount
    if (sclk_enable) begin // Output clock enabled
      if (rise_edge) begin // Count rising edges
        sclk_count <= sclk_count - ONE;
      end // if (rise_edge)
	end // if (sclk_enable)
    else begin
      sclk_count <= MAX_SCLK_COUNT; // Reset count when output clock is disabled
    end // else
  end : SclkCount
				
  // This block controls the transfer of data between the SPI Main and SPI Secondary device.
  // The input data is loaded into the spi buffer when the spi_go signal is received.
  // In write mode, the sdio output is always driven by the sdio register and 16-bits are written.
  // The sdio register is set to the current count sdio buffer value on the falling edge of the sclk.
  // In read mode, the sdio output is disabled after the eighth positive sclk edge.
  // Eight bits of data are written to the secondary device, then eight bits are read.
  always_ff @ (posedge i_clk) begin : TransferData
    if (~o_cs_n) begin	
      if ((sclk_count > 7) | (~read_write_n)) begin	// Always execute if writing. If read, only writing 8 bits
        sdio_enable <= '1;							// Enable to sdio output
        if (fall_edge) begin
          sdio <= sdio_buffer[sclk_count];          // Setup the bit on the falling edge
		end // if (fall_edge)
      end // if ((sclk_count > 7) | (~read_write_n))
      else begin                                    // Read data request - Read 8-bits after writing
        sdio_enable <= '0;                          // Disable the sdio output
        if (rise_edge) begin
          sdio_buffer[sclk_count] <= io_sdio;       // Register the bit on the rising edge
		end // if (rise_edge)
      end // else
    end // if (~os_c_n)
    else begin
      sdio_enable <= '0;                            // Disable the sdio output otherwise
      if (i_spi_go) begin
        sdio_buffer <= i_data;                      // Store input data if starting communication
      end // if (i_spi_go)
    end // else
  end : TransferData
  
endmodule

//   --- SPI Clock Generator Module ---
//
//    This module generates 'positive edge' and 'negative edge' signals
//    to be used as register enable signals to create a pseudo divided
//    down clock for logic within the FPGA. The module also outputs the
//    divided down clock for use by peripheral devices. The output clock
//    is delayed by one input clock to align it with the pseudo clock.
//    The parameters are such that a 50 MHz onboard FPGA clock will 
//    a create 1 MHz output clock. This is intended for SPI communication.
//
module spi_clock_generator (
  // -- System inputs --
  input 		 i_clk,     // FPGA clock - 50 MHz
  input 		 i_rst_n,   // Synchronous reset - active low
  // -- SPI Control Clocks --
  output logic o_rise_edge, // Rising edge signal - one clock pulse wide
  output logic o_fall_edge, // Falling edge signal - one clock pulse wide
  output logic o_clk_slow   // Divided down clock - 1 MHz
  );
	
  // -- Local constants --
  localparam MAX_COUNT = 5'd24; // Maximum count to divide 50 MHz to 1 MHz
  localparam ONE = 5'd1;        // 5-bit one for incrementing the count.
  
  // -- Local variables --
  logic [4:0] count; // Hold the count.
  logic clk_slow;    // Divided down clock. Store in a register to implement a delay

  // -- Sequential logic --
  always_ff @ (posedge i_clk) begin : CreateClk
    if (~i_rst_n) begin
      o_rise_edge <= '0;
      o_fall_edge <= '0;
      count <= '0;
      clk_slow <= '1;
    end // if (~i_rst_n)
    else begin
      count <= count + ONE;
      o_rise_edge <= '0;
      o_fall_edge <= '0;
      if (count == MAX_COUNT) begin
        o_rise_edge <= ~clk_slow;		// Create the rising edge pulse
        o_fall_edge <= clk_slow;		// Create the falling edge pulse
        count <= '0;				// Reset count
        clk_slow <= ~clk_slow;	// Invert the divided down clock - store in a register
      end // if (count == MAX_COUNT)
    end // else
  end : CreateClk

  // Delay the slow clock by one input clock cycle.
  always_ff @ (posedge i_clk) begin : DelayClk
    if (~i_rst_n) begin
      o_clk_slow <= '0;
	end // if (~i_rst_n)
    else begin
      o_clk_slow <= clk_slow;
	end // else
  end : DelayClk

endmodule

// NOTE: The code in this file has been updated for better readability.
// NOTE: The test bench has not been updated to coincide with the readability update.
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

