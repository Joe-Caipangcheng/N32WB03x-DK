1. Function description

    1. This routine shows the serial port upgrade application layer.
    2. It can receive serial port commands and jump to the Master Boot function.

2. Operating environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions

    System configuration;
        1. Clock source: HSI=64M, AHB=64M, APB1=32M, APB2=64M
        2. Port configuration:
            LED1: PB0
            LED2: PA6
        3. Data: USART1 PB6(TX) and PB7(RX), baud rate: 115200, 8 data bits, 1 stop bit, no parity bit, no hardware flow control

    usage method:
        Refer to Chapter 7.2 of the Firmware Upgrade User's Guide

4. Precautions
    Since the program will enter the sleep mode, the SWD will not be accessible. Press the RESET button to execute the program burning step within one second.