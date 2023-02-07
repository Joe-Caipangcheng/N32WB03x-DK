

1. Function description

    This test example demonstrates the basic communication between LPUART and USARTz by querying the detection mark. 
    USARTz can be USART2.LPUART sends TxBuffer1 data to USARTz, and USARTz receives data to RxBuffer2. At the same time, 
    USARTz sends TxBuffer2 data to LPUART, and LPUART receives data to RxBuffer1.
    Subsequently, the two groups of received data and sent data are compared respectively, and the comparison 
    results are stored in the TransferStatus1 variable and the TransferStatus2 variable.


2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB


3. Instructions for use

    The system clock configuration is as follows:
    -Clock source = HSI 64M
    -System clock = 64MHz
    
    The LPUART configuration is as follows:
    -Baud rate = 115200 baud
    -Word length = 8 data bits (fixed)
    -1 stop bit (fixed)
    -Verification control disabled
    -Hardware flow control disabled (RTS and CTS signals)
    -Receiver and transmitter enable
    
    The USARTz configuration is as follows:
    -Baud rate = 115200 baud
    -Word length = 8 data bits
    -1 stop bit
    -Verification control disabled
    -Hardware flow control disabled (RTS and CTS signals)
    -Receiver and transmitter enable
    
    The LPUART and USART pin connections are as follows:
    - LPUART_Tx.PB1   <------->   USART1_Rx.PB5
    - LPUART_Rx.PB2   <------->   USART1_Tx.PB4

    Log print: 
    - DEMO board PB6(TX), baud rate: 115200, 8 data bits, 1 stop bit, no parity bit, no hardware flow control
    
    Test steps and phenomena:
    -After the Demo is compiled in the KEIL environment, download it to the MCU
    -Reset operation, check the variables TransferStatus1 and TransferStatus2 in turn, 
      PASSED means the test passed, FAILED means the test is abnormal


4. Matters needing attention