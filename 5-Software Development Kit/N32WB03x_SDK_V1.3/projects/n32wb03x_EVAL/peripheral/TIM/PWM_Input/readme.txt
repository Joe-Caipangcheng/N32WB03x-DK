
1. Function description
    1. TIM3 CH2 capture pin calculates the duty cycle and frequency through the falling edge of CH1 and the rising edge of CH2
    
2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for Use
    System configuration;
        1. Clock source:
            HSI=64M,AHB=64M,APB1=32M,APB2=64M, TIM1 CLK=64M, TIM3 CLK=64M
         2. Interrupt:
                    TIM3 CC2 compare interrupt is turned on, priority level 1
         3. Port configuration:
                    PB5 is selected as the CH2 input of TIM3
                    PA6 is selected as IO output
         4. TIM:
                    TIM3 CH1 captures the falling edge of CH2 signals.TIM3 CH2 captures the rising edge of CH2. 
     Instructions:
         1. Open the debug mode after compiling, connect PA3 and PA7, and add the variables TIM3Freq and gOnePulsEn to the watch window
         2. The default gOnePulsEn=0, manually give gOnePulsEn=1 each time, then you can see that the values of the duty cycle and frequency captured are stored in the DutyCycle and Frequency variables.
4. Matters needing attention
    None