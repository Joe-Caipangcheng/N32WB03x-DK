
1. Function description
    1. TIM1 cycle gated TIM3

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for Use
    System configuration;
        1. Clock source:
            HSI=64M,AHB=64M,APB1=32M,APB2=64M, TIM1 CLK=64M, TIM3 CLK=64M
        2. Port configuration:
            PB8 is selected as CH1 output of TIM1
            PB7 is selected as CH1 output of TIM3
        3. TIM:
            TIM1 cycle triggers CH1 of gated TIM3, i.e. TIM3 is 10 times cycle in TIM1
Usage method:
    1. Open debugging mode after compilation and observe TIM1 CH1, TIM3 CH1 waveforms with oscilloscope or logic analyzer
         2. The cycle of TIM3 is 10 times that of TIM1
         
4. Matters needing attention
        By default, the PB6 and PB7 jumper caps of the development board are connected to the virtual serial port of NSLINK. If PB6 and PB7 are not used as serial ports in the project, and are used for other purposes, the serial port jumper caps must be unplugged.