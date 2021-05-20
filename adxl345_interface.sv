// ADXL345 Accelerometer Interface
// adxl345_interface.sv
// Created By: Logan Beams
// Date: 05/08/2021

// ----- Constants -----
// --- ADXL345 Register Address Map ---
parameter THRESH_TAP     = 6'h1D; // Tap threshold
parameter OFSX           = 6'h1E; // X-axis offset
parameter OFSY           = 6'h1F; // Y-axis offset
parameter OFSZ           = 6'h20; // Z-axis offset
parameter DUR            = 6'h21; // Tap duration
parameter LATENT         = 6'h22; // Tap latency
parameter WINDOW         = 6'h23; // Tap window
parameter THRESH_ACT     = 6'h24; // Activity window
parameter THRESH_INACT   = 6'h25; // Inactivity window
parameter TIME_INACT     = 6'h26; // Inactivity time
parameter ACT_INACT_CTL  = 6'h27; // Axis enable control for activity and inactivity detection
parameter THRESH_FF      = 6'h28; // Free-fall threshold
parameter TIME_FF        = 6'h29; // Free-fall time
parameter TAP_AXES       = 6'h2A; // Axis control for single/double tap
parameter ACT_TAP_STATUS = 6'h2B; // Source of single/double tap
parameter BW_RATE        = 6'h2C; // Data rate and power mode control
parameter POWER_CTL      = 6'h2D; // Power-saving features control
parameter INT_ENABLE     = 6'h2E; // Interrupt enable control
parameter INT_MAP        = 6'h2F; // Interrupt mapping control
parameter INT_SOURCE     = 6'h30; // Source of interrupts
parameter DATA_FORMAT    = 6'h31; // Data format control
parameter DATAX0         = 6'h32; // X-axis data 0
parameter DATAX1         = 6'h33; // X-axis data 1
parameter DATAY0         = 6'h34; // Y-axis data 0
parameter DATAY1         = 6'h35; // Y-axis data 1
parameter DATAZ0         = 6'h36; // Z-axis data 0
parameter DATAZ1         = 6'h37; // Z-axis data 1
parameter FIFO_CTL       = 6'h38; // FIFO control
parameter FIFO_STATUS    = 6'h39; // FIFO status

// --- Register Initialization Values ---
parameter INIT_BW_RATE     = 8'b00000101; // 800 Hz sample rate mode
parameter INIT_POWER_CTL   = 8'b00001000; // Enable measurement mode
parameter INIT_DATA_FORMAT = 8'b01000100; // Enable 3-wire SPI and left-justified data
parameter INIT_INT_ENABLE  = 8'b10000000; // Enable Data-Ready interrupts
parameter INIT_INT_MAP     = 8'b01111111; // Send only Data-Ready to INT1 pin
parameter INIT_OFSX        = 8'b00000101; // X-Offset - Add 5 LSBs
parameter INIT_OFSY        = 8'b00000100; // Y-Offset - Add 4 LSBs
parameter INIT_OFSZ        = 8'b00001000; // Z-Offset - Add 8 LSBs

// --- Read/Write and Multibit---
parameter READ  = 1'b1;
parameter WRITE = 1'b0;

// 					--- ADXL345_Interface ---
//    This module is used to interface with the ADXL345 3-axis accelerometer
//    onboard the DE10-Lite board. This module instantiates the SPI_Controller
//    to communicate with the ADXL345.
//
//    Upon reset, the module first writes all desired initialization values
//    to the ADXL345. The last value written enables measurement mode.
//    After intialization, the module continously reads the ADXL345 whenever
//    the device sends out a DATA_READY interrupt. Once all the data has been
//    read, it is registered to the output of the module.
//
//    The configuration data is stored in two modules that act as ROM.
//
//    The ADXL345 stores data for each axis in two registers each. This module
//    combines the data into one register for output. Additionally, the ADXL345
//    is set to a 10 bit data format, so the last six LSBs are always zero and
//    therefore discarded.
//
module adxl345_interface (
  // -- System inputs --
  input              i_clk,        // System clock
  input              i_rst_n,      // Synchronous reset - active low		
  // -- Data output --
  output logic [9:0] o_data_x,     // Combined sensor data output for the x-axis
  output logic [9:0] o_data_y,     // Combined sensor data output for the y-axis
  output logic [9:0] o_data_z,     // Combined sensor data output for the z-axis
  output logic       o_data_valid, // Pulse to signal valid output data (all axis updated)
  // -- Connections to secondary devices -- 
  output logic       o_sclk,       // Output clock to secondary device
  inout wire         io_sdio,      // Secondary device data input and output.
  output logic       o_cs_n,       // Chip select - active low. (Only one device selectable)
  input              i_int1        // Secondary device interrupt INT1
  );

  // -- Local parameters --
  localparam CONFIG_COUNT = 3'd7; // Number of writes to initially configure the ADXL345 (8-1=7)
  localparam READ_COUNT   = 3'd5; // Number of reads needed to read all the ADXL345 data (6-1=5)
  localparam ONE          = 3'd1; // 3-bit one
  localparam ZEROS        = 8'd0; // 8-bit zero
  localparam IDLE         = 1'b0; // SPI state
  localparam TRANSFER     = 1'b1; // SPI state

  // -- Variable declarations --
  logic        spi_go;          // Command to start SPI
  logic        read_write_n;    // Command to read or write. 1 = Read, 0 = Write
  logic [15:0] spi_input_data;  // Data input to the SPI Controller
  logic [2:0]  config_count;    // Count the number of configuration writes
  logic [2:0]  read_count;      // Count the number of reads to obtain all the data
  logic        reading_data;    // High if reading data - ensures all data registers are read
  logic        config_done;     // Flag to exit configuration mode
  logic        spi_state;       // SPI controller state
  logic [5:0]  config_address;  // Register address to write to for configuration
  logic [7:0]  config_value;    // Value to write to the configuration register
  logic [5:0]  read_address;    // Register address to read data from
  logic        spi_data_valid;  // SPI Controller data valid
  logic        spi_idle;        // SPI Controller idle
  logic [7:0]  spi_output_data; // SPI Controller output data
  logic        data_ready;      // Data Ready interrupt

  logic [5:0][7:0] data_buffer; // Store the output data in a buffer

  // -- Continuous assignments --	
  assign data_ready = i_int1;                             // INT1 signals that there is new data to be read
  assign o_data_x = {data_buffer[4],data_buffer[5][7:6]}; // Put MSBs first. Last 6 LSB bits are zeros - don't need
  assign o_data_y = {data_buffer[2],data_buffer[3][7:6]}; // Put MSBs first. Last 6 LSB bits are zeros - don't need
  assign o_data_z = {data_buffer[0],data_buffer[1][7:6]}; // Put MSBs first. Last 6 LSB bits are zeros - don't need
  assign spi_input_data[15:14] = {read_write_n, 1'b0};    // Second bit is MultiRead flag. Will not use.

  // -- Module instantations --
  config_rom     cr   (.i_address(config_count), .o_config_address(config_address), .o_config_value(config_value));
  read_rom       rr   (.i_address(read_count), .o_read_address(read_address));
  spi_controller spic (.i_clk, .i_rst_n, .o_data_valid(spi_data_valid), .i_spi_go(spi_go), .i_read_write_n(read_write_n),
                       .o_idle(spi_idle), .i_data(spi_input_data), .o_data(spi_output_data), .o_sclk, .io_sdio, .o_cs_n);

  // -- Sequential Logic --
  // This block controls the SPI cntroller used to communicate with the ADXL345.
  // If the SPI controller is in an IDLE state, an spi_go signal is sent along with
  // the required data an whether to read/write. Initialization data is first written
  // to the ADXL345. A counter is used to track when the initialization is complete.
  // Then, data is read from the ADXL345 every time it sends out a DATA_READY interrupt.
  // The ADXL345 DATA_READY interrupt clears after the first data register is read, so
  // a counter is used to ensure all six data registers are read from. Once all registers
  // have been read, a output data valid flag is set.
  always_ff @ (posedge i_clk) begin : SpiControl
    if (~i_rst_n) begin
      config_count <= CONFIG_COUNT;
      read_count <= READ_COUNT;
      spi_go <= '0;
      spi_state <= IDLE;
      data_buffer <= '0;
      o_data_valid <= '0;
      config_done <= '0;
      reading_data <= '0;
    end // if (~i_rst_n)
    else begin
      case (spi_state)
        IDLE : begin
          o_data_valid <= '0;
          if (config_done) begin : RunMode
            if (data_ready | reading_data) begin
              spi_input_data[13:0] <= {read_address, ZEROS};
              read_write_n <= READ;
              spi_go <= '1;
              spi_state <= TRANSFER;
            end // if (data_ready)
          end : RunMode
          else begin : ConfigMode
            spi_input_data[13:0] <= {config_address, config_value};
            read_write_n <= WRITE;
            spi_go <= '1;
            if (~|config_count) begin
              config_done <= '1;
            end else begin
              config_count <= config_count - ONE;
            end // if/else (~|config_count)
            spi_state <= TRANSFER;
          end : ConfigMode
        end // IDLE
        TRANSFER : begin
          spi_go <= '0;
          if (spi_data_valid) begin
            data_buffer[read_count] <= spi_output_data;
            if (~|read_count) begin
              o_data_valid <= '1;
              reading_data <= '0;
              read_count <= READ_COUNT;
            end // if (~|read_count)
            else begin
              read_count <= read_count - ONE;
              reading_data <= '1;
            end // else
          end // (spi_data_valid)
          if (~spi_go & spi_idle) begin
            spi_state <= IDLE;
		  end // if (~spi_go && spi_idle)
        end // TRANSFER
      endcase
    end // else	
  end : SpiControl
	
endmodule

// 	-- Configuration ROM --
// This module contains the register addresses and initialization values
// for configuring the ADXL345. This is set up as a ROM. It is addressed using
// the configuration counter, and returns the current address and value.
module config_rom (
  input        [2:0] i_address,
  output logic [5:0] o_config_address,
  output logic [7:0] o_config_value
  );

  logic [7:0] CONFIG_VALUE [7:0];
  logic [5:0] CONFIG_ADDRESS [7:0];

  assign CONFIG_VALUE[7]   = INIT_OFSX;
  assign CONFIG_VALUE[6]   = INIT_OFSY;
  assign CONFIG_VALUE[5]   = INIT_OFSZ;
  assign CONFIG_VALUE[4]   = INIT_BW_RATE;
  assign CONFIG_VALUE[3]   = INIT_INT_MAP;
  assign CONFIG_VALUE[2]   = INIT_INT_ENABLE;
  assign CONFIG_VALUE[1]   = INIT_DATA_FORMAT;
  assign CONFIG_VALUE[0]   = INIT_POWER_CTL;
  assign CONFIG_ADDRESS[7] = OFSX;
  assign CONFIG_ADDRESS[6] = OFSY;
  assign CONFIG_ADDRESS[5] = OFSZ;
  assign CONFIG_ADDRESS[4] = BW_RATE;
  assign CONFIG_ADDRESS[3] = INT_MAP;
  assign CONFIG_ADDRESS[2] = INT_ENABLE;
  assign CONFIG_ADDRESS[1] = DATA_FORMAT;
  assign CONFIG_ADDRESS[0] = POWER_CTL;

  assign o_config_address = CONFIG_ADDRESS[i_address];
  assign o_config_value   = CONFIG_VALUE[i_address];

endmodule

// -- Read ROM --
// This module contains the register addresses the contain the desired data
// in the ADXL345. This is set up as a ROM. It is addressed using
// the read counter, and returns the current register address.
module read_rom (
  input        [2:0] i_address,
  output logic [5:0] o_read_address
  );

  logic [5:0] READ_ADDRESS [5:0];

  assign READ_ADDRESS[5] = DATAX0;
  assign READ_ADDRESS[4] = DATAX1;
  assign READ_ADDRESS[3] = DATAY0;
  assign READ_ADDRESS[2] = DATAY1;
  assign READ_ADDRESS[1] = DATAZ0;
  assign READ_ADDRESS[0] = DATAZ1;

  assign o_read_address = READ_ADDRESS[i_address];

endmodule

/*
module ADXL345_Interface_TB();
  logic i_clk, i_rst_n, o_data_valid, o_sclk, o_cs_n, i_int1;
  logic [9:0] x_data, y_data, z_data;
  logic [9:0] x_data_save, y_data_save, z_data_save;
  wire io_sdio;
  logic io_sdio_driver;

  adxl_interface dut (i_clk, i_rst_n, x_data, y_data, z_data, o_data_valid, o_sclk, io_sdio, o_cs_n, i_int1);
  assign io_sdio = io_sdio_driver;

  initial begin
    i_int1='0; io_sdio_driver=1'bz;
    x_data_save='0; y_data_save='0; z_data_save='0;
    i_rst_n='0; #10; i_rst_n='1; #5;
    repeat(5) @(posedge o_cs_n);
    repeat(10) @(posedge i_clk);
    i_int1='1; #5;
    while(!o_data_valid) begin
      repeat(8) @(posedge o_sclk);
      @(negedge o_sclk) io_sdio_driver<='1;
      repeat (7) @(negedge o_sclk) io_sdio_driver<=~io_sdio_driver;
      @(posedge o_cs_n) io_sdio_driver<=1'bz;
    end
    i_int1<='0;
    x_data_save<=x_data;
    y_data_save<=y_data;
    z_data_save<=z_data;
    #5;
  end

  initial begin
    i_clk='0; #5;
    repeat(10000)
      repeat(2) begin i_clk=~i_clk; #5; end
  end
endmodule
*/