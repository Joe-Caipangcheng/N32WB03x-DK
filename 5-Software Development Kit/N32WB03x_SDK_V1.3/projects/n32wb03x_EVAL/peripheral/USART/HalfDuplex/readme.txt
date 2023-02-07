
1. Function description

    This test example demonstrates the basic communication between USARTy and USARTz by querying and detecting identifications in half-duplex mode.
    First, USARTy sends TxBuffer1 data to USARTz, and USARTz receives data and stores it in RxBuffer2.
    Subsequently, USARTz sends TxBuffer2 data to USARTy, and USARTy receives data to RxBuffer1.
    Finally, compare the two groups of received data and sent data respectively, and store the comparison results in the TransferStatus1 variable
And the TransferStatus2 variable.
    USARTy and USARTz can be USART1 and USART2.


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
    -Half-duplex mode enabled
    
    The USART pin connections are as follows:
    - USART1_Tx.PB6    <------->   USART2_Tx.PB4

    
    Test steps and phenomena:
    -After the Demo is compiled in the KEIL environment, download it to the MCU
    -After resetting the operation, check the variables TransferStatus1 and TransferStatus2 in turn, 
      PASSED means the test passed, FAILED means the test is abnormal


4. Matters needing attention