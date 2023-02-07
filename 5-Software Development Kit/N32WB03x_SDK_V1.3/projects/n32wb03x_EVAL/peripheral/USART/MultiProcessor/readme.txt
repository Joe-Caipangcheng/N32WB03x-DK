

1. Function description

    This test example demonstrates how to use the USART multiprocessor mode. USARTy and USARTz can be USART1 and USART2.
    First, set the addresses of USARTy and USARTz to 0x1 and 0x2 respectively. USARTy continuously sends the character 0x33 to 
USARTz. USARTz receives 0x33 and flips the pins of LED1, LED2, and LED3.
    Once the KEY1_INT_EXTI_LINE line detects the rising edge, the EXTI0 interrupt is generated. In the EXTI0_IRQHandler interrupt 
processing function (the ControlFlag = 0), the USARTz enters the silent mode. In the silent mode, the LED pin stops toggling until 
the KEY1_INT_EXTI_LINE line detects the rising edge ( the ControlFlag = 1). In the EXTI0_IRQHandler interrupt processing function, 
USARTy sends address 0x102 to wake up USARTz. The LED pin restarts toggling.


2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for use

    The system clock configuration is as follows:
    -Clock source = HSI 64M
    -System clock = 64MHz
    
    The USARTy configuration is as follows:
    -Baud rate = 115200 baud
    -Word length = 9 data bits
    -1 stop bit
    -Verification control disabled
    -Hardware flow control disabled (RTS and CTS signals)
    -Receiver and transmitter enable
    
    
    The USART pin connections are as follows:
    - USART1_Tx.PB6    <------->   USART2_Rx.PB5
    - USART1_Rx.PB7    <------->   USART2_Tx.PB4    
    
    KEY1_INT_EXTI_LINE.PB1    <------->    BUTTON1
    
    LED1    <------->   PB0
    LED2    <------->   PA6

    
    Test steps and phenomena:
    -After the Demo is compiled in the KEIL environment, download it to the MCU
    -Reset operation, observe whether LED1~2 are blinking
    -Press the KEY button and observe if LED1~2 stop flashing
    -Press the KEY button again and observe whether LED1~2 resume flashing


4. Matters needing attention