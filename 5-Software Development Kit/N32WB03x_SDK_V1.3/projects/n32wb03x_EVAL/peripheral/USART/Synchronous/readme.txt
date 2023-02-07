
1. Function description

    This test example demonstrates the basic communication between USARTy (synchronous mode) and SPIy by querying the detection identifier.
USARTy and SPIy can be USART1 and SPI1, USART3 and SPI1 or USART2 and SPI2.
    First, through the TXC detection flag, USARTy sends TxBuffer1 data to SPIy, while SPIy receives data, it queries the RNE detection flag, and the 
received data is stored in RxBuffer1.
    Subsequently, SPIy sends TxBuffer2 data to USARTy by querying the TE detection flag. USARTy uses the RXDNE detection flag to receive data 
and stores it in RxBuffer2.
    Finally, compare the two groups of receiving and sending data respectively, and store the comparison results in the TransferStatus1 variable 
and the TransferStatus2 variable.


2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for use

    The system clock configuration is as follows:
    -Clock source = HSI 64M
    -System clock = 64MHz
    
    The USART configuration is as follows:
    -Baud rate = 115200 baud
    -Word length = 8 data bits
    -1 stop bit
    -Verification control disabled
    -Hardware flow control disabled (RTS and CTS signals)
    -Receiver and transmitter enable
    -Clock enable
    -Clock polarity: keep high when not sending out
    -Clock phase: sample the first data on the second clock edge
    -The last clock pulse: the clock pulse of the last data is output from CK
    
    The SPI configuration is as follows:
    -Direction = "Two-wire two-way" mode
    -Mode = slave mode
    -Data size = 8-bit data frame
    -CPOL = In idle state, the clock stays high
    -CPHA = data sampling starts from the second clock edge
    -NSS = Enable software management from the device
    -1st bit = 1st bit is LSB
    
    
    The USART pin connections are as follows:
    - USART1_Tx.PB6    <------->   SPI1_MOSI.PA2
    - USART1_Rx.PB7    <------->   SPI1_MISO.PA3
    - USART1_Clk.PA6   <------->   SPI1_SCK.PA1
    
    Test steps and phenomena:
    -After the Demo is compiled in the KEIL environment, download it to the MCU
    -Reset operation, check the variables TransferStatus1 and TransferStatus2 in turn, 
      PASSED means the test passed, FAILED means the test is abnormal


4. Matters needing attention

    the MCU_TX and MCU_RX jumper cap of the development board NS-LINK needs to be disconnected first