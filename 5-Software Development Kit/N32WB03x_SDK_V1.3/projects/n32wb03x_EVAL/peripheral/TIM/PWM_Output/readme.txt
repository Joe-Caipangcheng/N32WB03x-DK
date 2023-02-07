
1. Function description
     1. TIM3 CH1 CH2 CH3 CH4 outputs PWM with the same frequency and different duty cycles

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for Use
    System configuration;
        1. Clock source:
            HSI=64M,AHB=64M,APB1=32M,APB2=64M, TIM3 CLK=64M
         2. Port configuration:
                    PB4 is selected as the CH1 output of TIM3
                    PB5 is selected as the CH2 output of TIM3
                    PB6 is selected as the CH3 output of TIM3
                    PB7 is selected as the CH4 output of TIM3
         3. TIM:
                    TIM3 CH1 CH2 CH3 CH4 has the same period, and the duty cycle is not equal
     Instructions:
         1. After compiling, turn on the debug mode, use an oscilloscope or logic analyzer to observe the waveforms of TIM3 CH1, CH2, CH3, CH4
         2. After the program runs, 4 PWM signals with equal period and different duty cycle are generated
4. Matters needing attention
    None