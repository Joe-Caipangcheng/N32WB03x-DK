
1. Function description
    1. TIM1 output 3 complementary waveforms

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for Use
    System configuration;
        1. Clock source:
            HSI=64M,AHB=64M,APB1=32M,APB2=64M,TIM1 CLK=64M
        2. Port configuration:
                     PB8 is selected as TIM1 CH1 output
                     PB9 is selected as TIM1 CH2 output
                     PB10 is selected as TIM1 CH3 output
                     PB12 is selected as TIM1 CH1N output
                     PB13 is selected as TIM1 CH2N output
                     PA4  is selected as TIM1 CH3N output
                     PA6  is selected as TIM1 Breakin input
        3. TIM:
            TIM1 6 Complementary with dead-time, with IOM Brake
    Instructions:
        1. Open debugging mode after compilation and observe TIM1 waveform with oscilloscope or logic analyzer
        2. 3 complementary waveforms can be observed, and the output of PB12 pin can be turned off when the pin is high
4. Matters needing attention
    TIM3_CH3N is PA4 which is for SWDCLK also, it will delay 2s when power on and then change PA4 to TIM3_CH3N.