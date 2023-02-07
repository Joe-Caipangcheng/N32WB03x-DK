

1. Function description
    This routine provides a DMA usage for transferring data between I2C peripheral and RAM.

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for use
    1.Clock source: HSI 64M
    2.DMA channels: DMA_CH4, DMA_CH5
    3. SPI1 configuration:
        SCK   -->  PA1
        MISO  -->  PA3
        MOSI  -->  PA2
        Full duplex
        Main mode
        8 bit transmission
        Polarity: start at low/second edge
        Piece of software to choose
        Big end in front MSB

    4. SPI2 Configuration:
        SCK   -->  PB1 
        MISO  -->  PB3
        MOSI  -->  PB2
        Full duplex
        From the pattern
        8 bit transmission
        Polarity: start at low/second edge
        Piece of software to choose
        Big end in front MSB

    5.USART1 configuration:
        TX  -->  PB6
        Baud rate: 115200
        Data bit: 8 bits
        Stop bit: 1bit
        No check    

    6.Test steps and phenomena
        1. Compile download code reset run
        2. View the printed information from the serial port and verify the result

4. Precautions
    None
