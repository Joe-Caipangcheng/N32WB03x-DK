

1. Function description

    This test example demonstrates the basic communication between LPUART of two demo board using hardware flow control.
    First, Board1_LPUART uses CTS to send TxBuffer1 data, Board2_LPUART uses RTS to receive data and stores it in RxBuffer1; 
    Then, compare the received data with the sent data, and the comparison results are stored in the variables TransferStatus respectively.
    Note: Board1_LPUART is example at LPUART\HardwareFlowCtrl_ok\Transmit_CTS

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB


3. Instructions for use

    The system clock configuration is as follows:
    -Clock source = HSI 64M
    -System clock = 64MHz
    
    Board1_LPUART configuration is as follows:
    -Baud rate = 9600 baud
    -Word length = 8 data bits (fixed)
    -1 stop bit (fixed)
    -Verification control disabled
    -CTS hardware flow control enable
    -transmitter enable
   
    Board2_LPUART configuration is as follows:
    -Baud rate = 9600 baud
    -Word length = 8 data bits
    -1 stop bit
    -Verification control disabled
    -RTS hardware flow control enable)
    -Receiver enable
    
    
    The LPUART and USART pin connections are as follows:
    - Board1_LPUART_Tx.PB1    <------->   Board2_LPUART_Rx.PB2
    - Board1_LPUART_CTS.PB3   <------->   Board2_LPUART_RTS.PB0  

    Log print: 
    - DEMO board PB6(TX), baud rate: 115200, 8 data bits, 1 stop bit, no parity bit, no hardware flow control

    Test steps and phenomena:
    -After the demo is compiled in the KEIL environment, download it to the MCU
    -Reset operation, check the variables TransferStatus in log,
      PASSED(1) means the test passed, FAILED(0) means the test is abnormal


4. Matters needing attention