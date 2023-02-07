
1. Function description
    1. Trigger the calendar printout through the EXTI line.


2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for use

    System Configuration;
        1. RTC clock source: LSI
        2. Interrupt: TIM3 Interrupt 1Hz
        3. Serial port configuration:
                    - Serial port is USART1 (TX: PB6 RX: PB7):
                    - Data bits: 8
                    - Stop bit: 1
                    - Parity: none
                    - Baud rate: 115200


    Instructions:
        After compiling under KEIL and burning to the evaluation board. After power-on, the serial port will print the corresponding calendar time every 1S.


4. Matters needing attention
    none
