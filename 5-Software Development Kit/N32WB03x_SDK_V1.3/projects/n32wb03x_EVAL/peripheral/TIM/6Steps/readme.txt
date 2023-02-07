
1. Function description
    1. systick triggers TIM1 for 100ms to output 6-step commutation waveform

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for use
    System Configuration;
        1. Clock source:
                    HSI=64M,AHB=64M,APB1=32M,APB2=64M,TIM1 CLK=64M
        2. Interrupt:
                    TIM1 COM event interrupt on, priority 1
                    Systick 100ms interrupt, priority 0
        3. Port configuration:
                    PB8 is selected as TIM1 CH1 output
                    PB9 is selected as TIM1 CH2 output
                    PB10 is selected as TIM1 CH3 output
                    PB12 is selected as TIM1 CH1N output
                    PB13 is selected as TIM1 CH2N output
                    PA4  is selected as TIM1 CH3N output
                    PA6  is selected as TIM1 Breakin input
        4. TIM:
                    TIM1 6-channel complementary freeze output mode, no brake, open COM interrupt
    Instructions:
        1. Open the debug mode after compiling, and observe the output waveform of TIM1 with an oscilloscope or logic analyzer
        2. The systick triggers the COM interrupt every 100ms, and outputs the 6-step commutation waveform of AB AC BC BA CA CB in the COM interrupt of the TIM
        
4. Matters needing attention
       TIM3_CH3N is PA4 which is for SWDCLK also, it will delay 2s when power on and then change PA4 to TIM3_CH3N.
