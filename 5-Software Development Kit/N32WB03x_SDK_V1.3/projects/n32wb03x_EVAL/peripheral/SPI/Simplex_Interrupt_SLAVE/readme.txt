
1. Function description
    1. SPI slave mode single-wire interrupt receive data

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for use
    System Configuration;
        1. SystemClock：64MHz
        2. GPIO: (Master mode DEMO board) SPI1: SCK--PA1, MOSI--PA2
                 (Slave mode DEMO board)  SPI1: SCK--PA1, MISO--PA3
        3. Log print: slave mode DEMO board PB6(TX), baud rate: 115200
    Instructions:
        1. After compiling, download the program to reset and run;
        2. The slave mode DEMO board connects the serial port printing tool, power on, and check that the printing test is successful.


4. Matters needing attention
    Two demo boards are required, one board to program the master mode program, one board to program the slave mode program, 
    the two boards need to be powered on together, and connect the VCC and GND of the two boards;
    The "single wire" data lines are MOSI pins on the master side and MISO pins on the slave side.