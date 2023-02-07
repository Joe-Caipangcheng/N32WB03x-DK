  

1. Function description

    IWDG reset function.
    
2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB


3. Instructions for use

    System Configuration:
       1. IWDG clock source: LSI/128
       2. Timeout value: 300ms (4ms*75)
       3. light Indicator: LED1(PB0) LED2(PA6)

    Test steps and phenomenon：
       1. Compile and download the code to reset and run.The indicator LED2 
          keeps flashing. It means that IWDG feeds the dog normally and the code runs normally.
       2. Change the parameter of the Delay() function from 250 to 350 or more, the whole system
          will always be in the reset state, and LED1 will be on.

        
4. Matters needing attention
    1. If you simulate through the programmer, you need to enable DBG_ConfigPeriph(DBG_IWDG_STOP,ENABLE).
    2. 300ms is the timeout value obtained by theoretical calculation. In practice, the LSI clock has deviation, 
       so the parameters of the delay function are determined according to the actual situation.The skew range of
       the LSI clock is detailed in the data sheet.