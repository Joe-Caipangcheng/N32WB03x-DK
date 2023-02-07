
1. Function description
    1. SPI single-wire interrupt send and receive data

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for use
    System Configuration;
        1. SystemClock: 64MHz
        2. GPIO：SPI1: SCK--PA1, MOSI--PA2
                 SPI2: SCK--PB1, MISO--PB3
        3. Interrupt: SPI1 interrupt entry function SPI1_IRQHandler, SPI2 interrupt entry function SPI2_IRQHandler
    Instructions:
        1. After compiling, download the program to reset and run;
        2. When SPI1 has data to send, it enters the SPI1_IRQHandler interrupt function to send, and when SPI2 has data to receive,
            it enters the SPI2_IRQHandler interrupt function to receive. After the data transmission is completed, check the TransferStatus status as PASSED;

4. Matters needing attention
    The "single wire" data lines are MOSI pins on the master side and MISO pins on the slave side
