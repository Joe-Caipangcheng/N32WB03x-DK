
1. Function description

    This example demonstrates the basic communication between USART and SmartCard.


2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for use

    The system clock configuration is as follows:
    -Clock source = HSI 64M
    -System clock = 64MHz
    
    The USART pin connections are as follows:
    - USART2_Tx.PB4    <------->  SCDSIO
    - USART2_CK.PB13   <------->  SCDCLK
    - GPIO_PA0         <------->  SCDRST

    Log print: 
        DEMO board PB6(TX), baud rate: 115200, 8 data bits, 1 stop bit, no parity bit, no hardware 

    Test steps and phenomena:
    -After the Demo is compiled in the KEIL environment, download it to the MCU
    -Reset operation, check the log if the USART2 read SmartCard success.


4. Matters needing attention