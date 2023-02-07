
1. Function description

    This test example demonstrates how LPUART wakes up STOP mode through communication with PC.
     The MCU enters the STOP mode, and the PC sends the correct data to wake up the MCU. When the correct wake-up event is recognized, the WUF interrupt will trigger and wake up the MCU.
     In order to confirm whether to wake up, the MCU will send a confirmation message to the PC, and the PC will check whether it is the expected message.
     The process will be repeated 4 times to verify different wake-up events:
     1 Start bit detection, such as sending wake-up data ‘5A’ (Hex)
     2 The receiving buffer is not empty detection, such as sending wake-up data ‘5A’ (Hex)
     3 One configurable receive byte, such as sending wake-up data ‘5A’ (Hex)
     4 One programmable 4-byte frame, such as sending wake-up data ‘4B3C2D1E’ (Hex)
    

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB


3. Instructions for use

    The system clock configuration is as follows:
    -Clock source = HSI 64M
    -System clock = 64MHz
    -LPUART clock = LSE 32.768KHz
    
    The LPUART configuration is as follows:
    -Baud rate = 9600 baud
    -Word length = 8 data bits (fixed)
    -1 stop bit (fixed)
    -Verification control disabled
    -Hardware flow control disabled (RTS and CTS signals)
    -Receiver and transmitter enable
    
    
    The LPUART pin connections are as follows:
    - LPUART_Tx.PB1   
    - LPUART_Rx.PB2  

    
    Test steps and phenomena:
    -After the Demo is compiled in the KEIL environment, download it to the MCU
    -Reset operation, check the serial port print information, send the corresponding character through the serial port, wake up the MCU, and again
      Check the serial port print information (prompt "wake up"), this step needs to be repeated 4 times


4. Matters needing attention
    Please make sure the J15 and J16 connect the pin to crystal not the pogo pin when select the LSE as clock of low speed clock. 
    If if need to use LSI as low speed clock and LPUART wake up, please trim the LSI to 32.768KHz.