# Define Delays
set thold  5
set tsetup 5
set tprop  2

# Create Clocks
create_clock -period 20.000 [get_ports {MAX10_CLK1_50}]
create_generated_clock -name SCLK -source [get_ports {MAX10_CLK1_50}] -divide_by 50 -duty_cycle 50 [get_nets {adxl|spic|spicg|o_clk_slow}]
create_clock -period 1000.000 -name SCLK_VIRTUAL -waveform { 2 502 }
derive_clock_uncertainty

# Set delays for asynchronous inputs/outputs
# Define each as a false path
set_input_delay -clock { MAX10_CLK1_50 } 0 [get_ports { SW* KEY* INT* }]
set_output_delay -clock { MAX10_CLK1_50 } 0 [get_ports { CS_N LED* HEX* }]
set_false_path -from [get_ports { SW* KEY* INT* }]
set_false_path -to [get_ports { CS_N LED* HEX* }]

# Set delay for the output secondary clock
# False path between the FPGA clock and secondary clock
set_output_delay -clock { MAX10_CLK1_50 } -max 5 [get_ports { SCLK }]
set_output_delay -clock { MAX10_CLK1_50 } -min -5 [get_ports { SCLK }]
set_false_path -from [get_clocks MAX10_CLK1_50] -to [get_clocks SCLK]

# Set input delay for the SDIO pin
# Based off of delayed SCLK (virtual clock), received by ADXL345
set_input_delay -clock { SCLK_VIRTUAL } -max $tprop [get_ports { SDIO }]
set_input_delay -clock { SCLK_VIRTUAL } -min 0 [get_ports { SDIO }]

# Set output delay for SDIO pin
set_output_delay -clock { SCLK } -max [expr { $tsetup + $tprop }]  [get_ports { SDIO }]
set_output_delay -clock { SCLK } -min [expr { $tprop - $thold }] [get_ports { SDIO }]
