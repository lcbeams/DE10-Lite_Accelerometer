# DE10-Lite_Accelerometer
SystemVerilog design to use the accelerometer on the DE10-Lite FPGA Development Board.

This project is an educational one to learn more about the SystemVerilog language and designing digital circuits using an FPGA.
The board used in this project is the Intel/Altera DE10-Lite MAX 10 FPGA Development Board.
This board has an onboard ADXL345 3-axis accelerometer, and this project is aimed at configuring the FPGA to communicate
with the ADXL345 to receive acceleration data, and to use that data for some purpose.

The ADXL345 can communicate via a 3- or 4-wire SPI interface. A 3-wire interface was chosen to gain familiarity with
using a tristate buffer in a design. Currently, the design simply reads the acceleration data from the accelerometer
and does not implement any of the other functions (free-fall, tap, etc.). 

The FPGA can succesfully communicate with the ADXL345 to configure the device and read the acceleration data. The raw acceleration
data (in units of g's, with the LSB equal to 4mg) is passed through a CORDIC algorithm to determine the angles of rotation (roll and pitch).
The CORDIC algorithim convert the rectangular acceleration vectors to polar coordinates to obtain the angle. The angles of interest are
the board's roll and pitch, so the algorithm is used to compute atan(x/z) and atan(y/z) in units of degrees.

A physical validation module has been developed to verify functionality of the design. The raw acceleration data is displayed to
the onboard LEDs and the angles of rotation are displayed to the onbard 7-segment displays. The onboard slide switches can be used
to toggle between displaying the x-, y-, or z-axis raw data and between displaying the roll or pitch angle.

The physical validation meets timing requirements during compilation in Quartus while using the onboard 50 MHz.
A Synopsis Design Constraints (SDC) file was written and included here that fully constrains the inputs and outputs.

Future additions will include a UART transmitter to send to data to another device (i.e. a PC). This will perhaps be used to send an
alert to the PC if the device triggers a free-fall or tap interrupt. The logic to handle the interrupts will need to be designed.

Updated 05/19/2021
