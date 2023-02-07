
1. Function description
    1. TIM1 output 3 complementary waveforms and 1 CH4 waveform

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
                    PB11 is selected as TIM1 CH1N output
                    PB12 is selected as TIM1 CH2N output
                    PB13 is selected as TIM1 CH3N output
                    PA4  is selected as TIM1 CH4 output

        3. TIM:
            TIM1 6 complementary outputs, CH4 output

    Instructions:
        1. Open debugging mode after compilation and use an oscilloscope or logic analyzer to observe the output waveform of TIM1
        2. Output waveform TIM1 3-way complementary plus one CH4

4. Matters needing attention
    TIM3_CH3N is PA4 which is for SWDCLK also, it will delay 2s when power on and then change PA4 to TIM3_CH3N.
