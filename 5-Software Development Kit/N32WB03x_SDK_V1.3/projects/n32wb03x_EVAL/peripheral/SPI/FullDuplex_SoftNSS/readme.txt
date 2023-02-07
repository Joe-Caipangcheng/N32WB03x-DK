
1. Function description
    1. SPI full duplex software NSS mode to send and receive data

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for use
    System Configuration;
        1. SystemClock：48MHz
        2. GPIO：SPI1: SCK--PA1、MISO--PA3、MOSI--PA2,
                 SPI2: SCK--PB1、MISO--PB3、MOSI--PB2，

    Instructions:
        1. After compiling, download the program to reset and run;
        2. SPI1 is initialized as a master, sending data, SPI2 is initialized as a slave, receiving data, 
        after the transmission is completed, check the data, and check that the status of TransferStatus1 
        and TransferStatus2 is PASSED； SPI2 is initialized as a master, sending data, SPI1 is initialized 
        as a slave, receiving data, after the transmission is completed, check the data, and check that 
        the status of TransferStatus3 and TransferStatus4 is PASSED;
        
4. Matters needing attention
    No