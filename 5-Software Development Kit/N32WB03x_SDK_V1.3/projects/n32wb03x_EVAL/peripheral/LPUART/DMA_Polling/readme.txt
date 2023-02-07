
1. Function description

    This test example demonstrates the basic communication between LPUART and USARTz through DMA (query). USARTz can be USART2.
     First, DMA transfers TxBuffer1 data to the LPUART send data register, and then the data is sent to USARTz. USARTz receives data, 
    and DMA moves the data from the USARTz receive data register to RxBuffer2.
    Compare the received and sent data, and store the comparison result in the TransferStatus2 variable.
     At the same time, DMA transfers TxBuffer2 data to the USARTz send data register, and then the data is sent to LPUART. LPUART 
    receives data, DMA moves data from LPUART receive data register to RxBuffer1.
    Compare the received and sent data, and the comparison result is stored in the TransferStatus1 variable.

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
    -DMA transmit mode and DMA receive mode enable
    
    The USART2 configuration is as follows:
    -Baud rate = 115200 baud
    -Word length = 8 data bits
    -1 stop bit
    -Verification control disabled
    -Hardware flow control disabled (RTS and CTS signals)
    -Receiver and transmitter enable
    -DMA transmit mode and DMA receive mode enable
    
    The USART pin connections are as follows:
    -LPUART_Tx.PB1 <-------> USART2_Rx.PB5
    -LPUART_Rx.PB2 <-------> USART2_Tx.PB4

    Log print: 
    - DEMO board PB6(TX), baud rate: 115200, 8 data bits, 1 stop bit, no parity bit, no hardware flow control
    
    Test steps and phenomena:
    -After the Demo is compiled in the KEIL environment, download it to the MCU
    -Reset operation, check the variables TransferStatus1 and TransferStatus2 in turn, 
      PASSED means the test passed, FAILED means the test is abnormal


4. Matters needing attention