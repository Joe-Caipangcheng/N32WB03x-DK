
1. Function description
     1. TIM1 cycle gated TIM3

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for Use
    System configuration;
        1. Clock source:
            HSI=64M,AHB=64M,APB1=32M,APB2=64M,TIM1 CLK=64M
        2. Port configuration:
                    PB8 is selected as TIM1 CH1 output
                    PB4 is selected as TIM3 CH1 output
        3. TIM:
                    TIM1 cycle gating TIM3
    Instructions:
        1. Open debugging mode after compilation and observe TIM1 CH1, TIM3 CH1 waveforms with an oscilloscope or logic analyzer
        2. TIM3 4 times cycle TIM1
        
4. Matters needing attention
    None