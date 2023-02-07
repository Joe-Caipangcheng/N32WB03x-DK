
1. Function description
     1. TIM6 uses the update interrupt to generate timing rollover IO

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for Use
    System configuration;
        1. Clock source:
            HSI=64M,AHB=64M,APB1=32M,APB2=64M, TIM1 CLK=64M,TIM3 CLK=64M
         2. Interruption:
                     TIM6 update interrupt is turned on, priority level 1
         3. Port configuration:
                     PB6 is selected as IO output
         4. TIM:
                     TIM6 enables periodic interrupts
     Instructions:
         1. After compiling, turn on the debug mode, and use an oscilloscope or logic analyzer to observe the PB6 waveform
         2. After the program is running, the periodic interrupt of TIM6 comes to reverse the level of PB6
         
4. Matters needing attention
    By default, the PB6 and PB7 jumper caps of the development board are connected to the virtual serial port of NSLINK. If PB6 and PB7 are not used as serial ports in the project, and are used for other purposes, the serial port jumper caps must be unplugged.