

1. Function description
    1. Trigger the alarm interrupt by setting the alarm time.

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB


3. Instructions for use

    The system clock configuration is as follows:
    -Clock source = HSI 64M
    -System clock = 64MHz

3. Instructions for use

    System Configuration;
        1. RTC clock source: LSI
        2. Serial port configuration:
                    - Serial port is USART1 (TX: PB6 RX: PB7):
                    - Data bits: 8
                    - Stop bit: 1
                    - Parity: none
                    - Baud rate: 115200


    Instructions:
        After compiling under KEIL, burn it to the evaluation board, power on, and the serial port will print out the time set by the alarm clock.

4. Matters needing attention
    none