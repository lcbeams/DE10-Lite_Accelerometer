# DE10-Lite_Accelerometer
SystemVerilog design to use the accelerometer on the DE10-Lite FPGA Dev Board.

This project is an educational one to learn more about the SystemVerilog language and designing digital circuits using an FPGA.
The board used in this project is the Intel/Altera DE10-Lite MAX 10 FPGA Development Board.
This board has an ADXL345 3-axis accelerometer, and this project is aimed at configuring the FPGA to communicate
with the ADXL345 to receive data, and to use that data for some purpose.

The ADXL345 can communicate via an 3- or 4-wire SPI interface. A 3-wire interface was chosen to gain familiarity with
using a tristate buffer in a design. Currently, the design simply reads the acceleration data from the accelerometer,
and does not implement any of the other functions (free-fall, tap, etc.). So far, the FPGA can succesfully communicate with
and read data from the ADXL345, and a physical validation module has been written to display the data using the onboard LEDs.

Currently in progress are modules to convert the acceleration data (in units of g's) to degrees of rotation about the
x- and y-axis (roll and pitch). The method chosen to accomplish this is a CORDIC algorithim to conver the Cartesian 
acceleration vectors to polar coordinates, specifically to obtain the angle. The vectors of interest are {z,x} and {z,y}, so
as to compute the atan(x/z) and atan(y/z) in units of degrees.

Future additions will include a physical validation with a display of the angle of rotation to the onboard 7-segment displays,
and a UART transmitter to send to data to another device (i.e. a PC).

Updated 05/14/2021
