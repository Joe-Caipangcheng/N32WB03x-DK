
Function Description
     1. After TIM3 CH1 CH2 CH3 CH4 reaches the CC value, pull down the IO level of PB4 PB5 PB6 PB7 correspondingly.

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for Use
    System configuration;
        1. Clock source:
            HSI=64M,AHB=64M,APB1=32M,APB2=64M, TIM3 CLK=64M
         2. Interruption:
                    TIM3 compare interrupt is turned on, subpriority 1
         3. Port configuration:
                     PB4 is selected as IO output
                     PB5 is selected as IO output
                     PB6 is selected as IO output
                     PB7 is selected as IO output
         4. TIM:
                    TIM3 configures the comparison value of CH1, CH2, CH3, CH4, and turns on the comparison interrupt
     Instructions:
         1. Open the debug mode after compiling, and observe the waveforms of PA6 PA7 PA8 PA9 with an oscilloscope or logic analyzer
         2. After the timer runs to CC1 CC2 CC3 CC4, pull down the IO level of PA6 PA7 PA8 PA9 correspondingly
4. Matters needing attention
    By default, the PB6 and PB7 jumper caps of the development board are connected to the virtual serial port of NSLINK. If PB6 and PB7 are not used as serial ports in the project, and are used for other purposes, the serial port jumper caps must be unplugged.