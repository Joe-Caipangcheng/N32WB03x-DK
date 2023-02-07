
1. Function description
    1. I2S slave mode sends and receives data through DMA

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for use
    System Configuration;
        1. SystemClock: HSI 64MHz
        2. GPIO：(Master mode DEMO board) I2S1: WS--PA0, CK--PA1, SD--PA2
                 (Slave mode DEMO board)  I2S1: WS--PA0, CK--PA1, SD--PA2
        3. DMA: I2S1 transmission adopts DMA_CH3 channel, I2S1 reception adopts DMA_CH2 channel;
        4. Log print: DEMO board PB6(TX), baud rate: 115200, 8 data bits, 1 stop bit, no parity bit, no hardware flow control

    Instructions:
        1. After compiling, download the program to reset and run;
        2. After the master mode DEMO board and slave mode DEMO board PA0, PA1, PA2 are connected, 
           the master mode DEMO board is connected to the serial port printing tool, power on, 
           first press and hold the master mode DEMO board reset button, and then press the slave mode DEMO board reset button, 
           after the slave mode DEMO board has run, release the reset button of the master mode DEMO board and then check the printing test is successful;

4. Matters needing attention
    Two demo boards are required, one board to burn the master mode program, one board to burn the slave mode program, connect the VCC and GND of the two boards