
1. Function description

    This test example demonstrates the basic communication between USARTy and USARTz through DMA (interrupt). USARTy and USARTz can be USART1 and USART2.
    First, DMA transfers the TxBuffer1 data to the USARTy send data register. After the transfer is completed, a DMA interrupt is generated, and then the data is sent to 
USARTz. USARTz receives data, DMA moves data from USARTz receive data register to RxBuffer2, and generates DMA interrupt after completion.
    At the same time, DMA transfers TxBuffer2 data to the USARTz send data register, the transfer is completed and a DMA interrupt is generated, and then the data is 
sent to LPUART. USARTy receives data, DMA moves the data from the LPUART receive data register to RxBuffer1, and generates a DMA interrupt after completion.
    Finally, compare the two groups of receiving and sending data respectively, and store the comparison results in the TransferStatus1 variable and the TransferStatus2 
variable.

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
    -DMA transmission mode is enabled, DMA reception mode is disabled
    
    The USART pin connections are as follows:
    - USART1_Tx.PB6    <------->   USART2_Rx.PB5
    - USART1_Rx.PB7    <------->   USART2_Tx.PB4
    
    Test steps and phenomena:
    -After the Demo is compiled in the KEIL environment, download it to the MCU
    -Reset operation, check the variables TransferStatus1 and TransferStatus2 in turn, 
      PASSED means the test passed, FAILED means the test is abnormal


4. Matters needing attention
    the MCU_TX and MCU_RX jumper cap of the development board NS-LINK needs to be disconnected first