
1. Function description

    1. This routine shows the OTA upgrade application layer.
    2. The dual bank upgrade function is realized.
    3. Realize the function of copying Image Update and jumping to Image Update in single bank upgrade.

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
        3. Log print: DEMO board PB1(TX), baud rate: 115200, 8 data bits, 1 stop bit, no parity bit, no hardware flow control
        4. Bluetooth BLE name: NATIONS

    usage method:
        Refer to Chapter 7.3 of the Firmware Upgrade User's Guide

4. Precautions
    Since the program will enter the sleep mode, the SWD will not be accessible. Press the RESET button to execute the program burning step within one second.