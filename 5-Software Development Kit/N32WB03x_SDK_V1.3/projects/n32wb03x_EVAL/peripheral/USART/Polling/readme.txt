
1. Function description

    This test case demonstrates the basic communication between USARTy and USARTz by querying the detection identification.
    First, USARTy sends TxBuffer1 data to USARTz, and USARTz receives data and stores it in RxBuffer1.
Compare the received data with the sent data, and the comparison result is stored in the TransferStatus variable.
    Subsequently, USARTz sends TxBuffer2 data to USARTy, and USARTy receives data to RxBuffer2.
Compare the received data with the sent data, and the result of the comparison is stored in the TransferStatus variable.
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
    
    The USART pin connections are as follows:
    - USART1_Tx.PB6    <------->   USART2_Rx.PB5
    - USART1_Rx.PB7    <------->   USART2_Tx.PB4 

    
    Test steps and phenomena:
    -After the Demo is compiled in the KEIL environment, download it to the MCU
    -Reset operation, check the variables TransferStatus in turn, 
      PASSED means the test passed, FAILED means the test is abnormal


4. Matters needing attention

    the MCU_TX and MCU_RX jumper cap of the development board NS-LINK needs to be disconnected first