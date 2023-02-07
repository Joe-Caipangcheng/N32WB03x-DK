
1. Function description
     1. TIM3 uses the CH1 CH2 CH3 CH4 CC value to generate a timing interrupt and flip the IO level

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for Use
    System configuration;
        1. Clock source:
            HSI=64M,AHB=64M,APB1=32M,APB2=64M, TIM1 CLK=64M,TIM3 CLK=64M
         2. Interruption:
             TIM3 compare interrupt is turned on, priority level 1
         3. Port configuration:
                     PB6 is selected as IO output
                     PB7 is selected as IO output
                     PB11 is selected as IO output
                     PB12 is selected as IO output
         4. TIM:
                     TIM3 output freeze mode, CH1 CH2 CH3 CH4 compare value interrupt
     Instructions:
         1. Open the debug mode after compiling, and observe the waveform of PB6,PB7,PB11,PB12 with an oscilloscope or logic analyzer
         2. After the program runs, after the corresponding channel reaches the comparison value, the comparison value accumulates and flips the corresponding IO port level
Matters needing attention
    By default, the PB6 and PB7 jumper caps of the development board are connected to the virtual serial port of NSLINK. If PB6 and PB7 are not used as serial ports in the project, and are used for other purposes, the serial port jumper caps must be unplugged.