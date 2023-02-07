
1. Function description
    1. SPI read, write, erase W25Q128

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for use
    System Configuration;
        1. SystemClock：64MHz
        2. SPI1: NSS--PA4, SCK--PA5, MISO--PA6, MOSI--PA7
    Instructions:
        1. After compiling, download the program to reset and run;
        2. Read the ID of W25Q128 through SPI1, then write the data, then read it out, compare the read and write data, 
           check that the status of TransferStatus1 is PASSED, then erase the block, and check that the erase block is normal;

4. Matters needing attention
    Connect the SPI1 interface of the development board to the W25Q128 of the N32G457QE_EVB development board, 
    the two development boards need to share the ground.
