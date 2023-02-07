1. Function description

    Bluetooth DTM mode with HCI commands supported.

2. Operating environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions

    System configuration;
        1. Clock source: HSI=64M, AHB=64M, APB1=32M, APB2=64M
        2. Port configuration:
            LED1: PB0
            LED2: PA6
            KEY1: PB1
            KEY2: PB2
        3. command port: DEMO board PB6(TX) PB7(RX), baud rate: 115200, 8 data bits, 1 stop bit, no parity bit, no hardware flow control

    usage method:
        1. Burn to development board after compilation
        2. Connect the UART com port to Bluetooth protocol tester, such as CMW270.
        3. Start TX or RX test with Bluetooth protocol tester.

