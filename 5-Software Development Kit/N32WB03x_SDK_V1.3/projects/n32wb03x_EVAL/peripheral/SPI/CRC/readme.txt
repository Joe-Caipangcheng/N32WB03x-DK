
1. Function description
    1. SPI sends and receives data for CRC check

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for use
    System Configuration;
        1. SystemClock: HSI 64MHz
        2. GPIO：SSPI1: SCK--PA1、 MISO--PA3、MOSI--PA2,
                 SPI2: SCK--PB1、 MISO--PB3、MOSI--PB2，

    Instructions:
        1. After compiling, download the program to reset and run;
        2. SPI1, SPI2 send and receive data at the same time, after the transmission is completed, send CRC data, check the data and CRC value, Check that the status of TransferStatus1 and TransferStatus2 is PASSED, and then check the CRC value;
        
4. Matters needing attention
    No