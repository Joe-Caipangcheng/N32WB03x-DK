
1. Function description

    1. This routine shows the Image Update application layer of a single bank in OTA upgrade.
    2. Realize the function of upgrading the Image Update layer in a single bank.

2. Operating environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions

    System configuration;
        1. Clock source: HSI=64M, AHB=64M, APB1=32M, APB2=64M
        2. Bluetooth BLE name: ImageUpdate


    usage method:
        Refer to Chapter 7.4 of the Firmware Upgrade User's Guide

4. Precautions
    Since the program will enter the sleep mode, the SWD will not be accessible. Press the RESET button to execute the program burning step within one second.