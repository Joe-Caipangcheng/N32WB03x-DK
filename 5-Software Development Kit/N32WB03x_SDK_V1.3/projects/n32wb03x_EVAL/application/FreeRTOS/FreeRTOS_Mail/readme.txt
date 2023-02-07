1. Function description

    This example mainly demonstrates the use of the tickless mode with low power consumption of FreeRTOS (in and out of the Idle mode of the kernel)

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

    usage method:
        1. Burn to development board after compilation
        2. Check LED blinking

4. Precautions