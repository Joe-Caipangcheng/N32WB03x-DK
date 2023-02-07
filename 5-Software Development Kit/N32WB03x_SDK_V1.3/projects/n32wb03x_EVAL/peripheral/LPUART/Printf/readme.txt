
1. Function description

     This test example demonstrates the basic communication between LPUART and PC by querying the detection mark.
     Redirect the printf function to LPUART, and use the printf function to output messages to the terminal.


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
    
     The LPUART pin connections are as follows:
     -LPUART_Tx.PB1
    
     Test steps and phenomena:
     -After the Demo is compiled in the KEIL environment, download it to the MCU
     -Reset operation, view serial port print information


4. Matters needing attention