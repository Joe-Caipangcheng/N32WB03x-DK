
1. Functional description
    1. TIM1 CH3 CH3N complementary signal changes duty cycle every 6 cycles

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for Use
    System configuration;
        1. Clock source:
            HSI=64M,AHB=64M,APB1=32M,APB2=64M,TIM1 CLK=64M
        2. Port configuration:
            PB8  selected as TIM1 CH1 Output
            PB12 selected as TIM1 CH1N Output
        3. TIM:
            TIM1 CH1 CH1N complementary output triggers DMA transmission every 6 cycles
        4. DMA:
            DMA1_ CH5 Channel circular mode handling 3 half-Word SRC_ Buffer[3] variable to TIM1 CCDAT3 register
     Instructions:
         1. After compiling, turn on the debug mode, use an oscilloscope or logic analyzer to observe the waveform of TIM1 CH1 CH1N
         2. Change the duty cycle of CH1 and CH1N once in 6 cycles of TIM1, and change cyclically

4. Matters needing attention
    None
