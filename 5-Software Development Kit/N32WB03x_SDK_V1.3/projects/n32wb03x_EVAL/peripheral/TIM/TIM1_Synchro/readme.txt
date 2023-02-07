
1. Function description
     1. TIM3 counts under TIM1 cycle

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for Use
    System configuration;
        1. Clock source:
            HSI=64M,AHB=64M,APB1=32M,APB2=64M, TIM1 CLK=64M,TIM3 CLK=64M
         2. Port configuration:
                    PB7 is selected as the CH1 output of TIM3
                    PB8 is selected as the CH1 output of TIM1
         3. TIM:
                    TIM1 CH1 periodically triggers the gate control of TIM3
     Instructions:
         1. Open the debug mode after compiling, and observe the waveforms of TIM1 CH1, TIM3 CH1,with an oscilloscope or logic analyzer
         2. After the program runs, The period of TIM3 is 8 times that of TIM1

4. Matters needing attention
    By default, the PB6 and PB7 jumper caps of the development board are connected to the virtual serial port of NSLINK. If PB6 and PB7 are not used as serial ports in the project, and are used for other purposes, the serial port jumper caps must be unplugged.