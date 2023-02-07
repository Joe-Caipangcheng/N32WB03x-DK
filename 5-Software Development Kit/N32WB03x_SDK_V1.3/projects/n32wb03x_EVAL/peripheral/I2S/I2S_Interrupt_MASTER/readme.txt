
1. Function description
    1. I2S master mode sends data via interrupt

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for use
    System Configuration;
        1. SystemClock: HSI 64MHz
        2. GPIO：(Master mode DEMO board) I2S1: WS--PA0, CK--PA1, SD--PA2
                 (Slave mode DEMO board)  I2S1: WS--PA0, CK--PA1, SD--PA2
        3. Interrupt: I2S1 interrupt entry function SPI1_IRQHandler
        4. Log print: DEMO board PB6(TX), baud rate: 115200, 8 data bits, 1 stop bit, no parity bit, no hardware flow control

    Instructions:
        1. After compiling, download the program to reset and run;
        2. The slave mode DEMO board enters the debug mode, first press and hold the master mode DEMO board reset button, 
           then the slave mode DEMO board runs at full speed,then release the reset button of the master mode DEMO board.
           
4. Matters needing attention
    Two demo boards are required, one board to program the master mode program, one board to program the slave mode program, 
    the two boards need to be powered on together, and connect the VCC and GND of the two boards.