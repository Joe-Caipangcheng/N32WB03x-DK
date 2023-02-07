
1. Function description

    This example demonstrates the basic communication between USARTy and USARTz using hardware flow control. USARTy and USARTz can be USART1 and USART2.
    First, USARTy uses CTS to send TxBuffer1 data, USARTz uses RTS to receive data and stores it in RxBuffer1; 
    Then, compare the received data with the sent data, and the comparison results are stored in the variables TransferStatus respectively.

    Note: test example with \USART\HardwareFlowCtrl_ok\Transmit_RTS

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for use

    The system clock configuration is as follows:
    -Clock source = HSI 64M
    -System clock = 64MHz
    
    The USARTy configuration is as follows:
    -Baud rate = 115200 baud
    -Word length = 8 data bits
    -1 stop bit
    -Verification control disabled
    -CTS hardware flow control enable
    -Transmitter enable
    
    The USARTz configuration is as follows:
    -Baud rate = 115200 baud
    -Word length = 8 data bits
    -1 stop bit
    -Verification control disabled
    -RTS hardware flow control enable
    -Receiver enable
    
    
    The USART pin connections are as follows:
    - USART1_Tx.PB6    <------->   USART2_Rx.PB5
    - USART1_CTS.PB9   <------->   USART2_RTS.PB11

    
    Test steps and phenomena:
    -After the Demo is compiled in the KEIL environment, download it to the MCU
    -Reset operation, check the variables TransferStatus in turn,  PASSED means the test passed and FAILED means the test is abnormal


4. Matters needing attention
    the MCU_TX and MCU_RX jumper cap of the development board NS-LINK needs to be disconnected first