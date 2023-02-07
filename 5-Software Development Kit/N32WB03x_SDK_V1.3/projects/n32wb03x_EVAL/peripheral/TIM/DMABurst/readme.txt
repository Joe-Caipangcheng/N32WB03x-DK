
1. Function description
     1. TIM1 changes the period and duty cycle at the same time after one cycle
     
2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for Use
    System configuration;
        1. Clock source:
            HSI=64M,AHB=64M,APB1=32M,APB2=64M,TIM1 CLK=64M
        2. Port configuration:
                     PA8 is selected as TIM1 CH1 output
        3. TIM:
            TIM1 CH1 output, periodically triggered DMA burst transmission, loading AR, REPCNT, CCDAT1 registers, changing duty cycle, period and repeat counter
        4. DMA:
                     The DMA1_CH5 channel in normal mode transfers 3 half-word SRC_Buffer[3] variables to the TIM1 DMA register
    Instructions:
        1. Open debugging mode after compilation and observe TIM1 CH 1 waveform with oscilloscope or logic analyzer
         2. After the first cycle of TIM1 is over, the following waveforms are the waveforms of changing cycle and duty cycle of DMA transport
4. Matters needing attention
    None
