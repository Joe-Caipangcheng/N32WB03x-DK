
1. Function description
    1. SPI master mode full duplex DMA send and single-wire receive data

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for use
    System Configuration;
        1. SystemClock：64MHz
        2. GPIO：(Master mode DEMO board) SPI1: SCK--PA1, MISO--PA3, MOSI--PA2
                 (Slave mode DEMO board)  SPI1: SCK--PA1, MISO--PA3, MOSI--PA2
        3. Log print: master mode DEMO board PB6(TX), baud rate: 115200
    Instructions:
        1. After compiling, download the program to reset and run;
        2. Connect the serial port printing tool, power on, and check that the print test is successful;


4. Matters needing attention
    Two demo boards are required, one board to program the master mode program, one board to program the slave mode program, 
    the two boards need to be powered on together, and connect the VCC and GND of the two boards;