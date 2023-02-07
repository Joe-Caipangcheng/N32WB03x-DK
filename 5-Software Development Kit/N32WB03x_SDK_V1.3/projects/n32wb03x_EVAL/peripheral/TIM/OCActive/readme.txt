
1. Function description
     1. After TIM3 CH1 CH2 CH3 CH4 reaches the CC value, it outputs ACTIVE level

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for Use
    System configuration;
        1. Clock source:
            HSI=64M,AHB=64M,APB1=32M,APB2=64M, TIM3 CLK=64M
         2. Port configuration:
                    PB4 is selected as TIM3 CH1 output
                    PB5 is selected as TIM3 CH2 output
                    PB6 is selected as TIM3 CH3 output
                    PB7 is selected as TIM3 CH4 output
                    PB8 is selected as IO output
     Instructions:
        1. Open the debug mode after compiling, and observe the waveforms of TIM3 CH1 CH2 CH3 CH4 with an oscilloscope or logic analyzer
        2. After the timer runs to CC1 CC2 CC3 CC4, the output of the corresponding channel becomes Active
4. Matters needing attention
     None