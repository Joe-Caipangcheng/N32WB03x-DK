
1. Function description
    1. TIM3 CH2 pin, CH2 rising edge, CH1 falling edge to calculate the frequency

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for Use
    System configuration;
        1. Clock source:
            HSI=64M,AHB=64M,APB1=32M,APB2=64M, TIM3 CLK=64M
        2. Interruption:
            TIM3 CH2 rising edge CH1 falling edge interrupt on, priority 1
        3. Port configuration:
            PB5 is selected TIM3 CH2 Input
            PA6 is selected IO Output
        4. TIM:
            TIM3 CH2 rising edge, CH1 falling edge capture interrupt is turned on
Usage method:
    1. Open debugging mode after compilation, connect PA3 to PA7, add variables TIM3Freq, gOnePulsEn to the watch window
    2. Default gOnePulsEn=0, each time you manually give gOnePulsEn=1, you can see the frequency value calculated by TIM3Freq
4. Matters needing attention
    None
