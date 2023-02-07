
1. Function description
    1. Trigger the interrupt by setting the wake-up time.
    2. Configure the IO output through the wake-up flag bit

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB


3. Instructions for use

    System Configuration;
        1. Periodic wake-up clock source: RTCCLK (1HZ)
        2. Serial port configuration:
                    - Serial port is USART1 (TX: PB6 RX: PB7):
                    - Data bits: 8
                    - Stop bit: 1
                    - Parity: none
                    - Baud rate: 115200


    Instructions:
        1. After compiling under KEIL, burn it to the evaluation board. After power-on, the serial port will print I am in rtc_wkup every 4s.
            
4. Matters needing attention
    none