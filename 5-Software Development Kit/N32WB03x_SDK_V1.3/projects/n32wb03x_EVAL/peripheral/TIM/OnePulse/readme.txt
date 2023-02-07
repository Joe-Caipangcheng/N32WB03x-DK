
1. Function description
     1. The rising edge of TIM3 CH2 triggers CH1 to output a single pulse


2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for Use
    System configuration;
        1. Clock source:
            HSI=64M,AHB=64M,APB1=32M,APB2=64M, TIM3 CLK=64M
         2. Port configuration:
                    PB6 is selected as the CH3 output of TIM3
                    PB5 is selected as the CH2 input of TIM3
                    PA6 is selected as IO output
         3. TIM:
                     TIM3 configures CH2 rising edge to trigger CH3 to output a single pulse
     Instructions:
         1. Open the debug mode after compiling, connect PA6 to PB5, and observe the waveform of CH3 of TIM3 with an oscilloscope or logic analyzer
         2. Add the variable gSendTrigEn to the watch window, the default gSendTrigEn=0, every time you manually modify gSendTrigEn=1, you will see a single pulse output on the TIM3 CH3 port

4. Matters needing attention
        By default, the PB6 and PB7 jumper caps of the development board are connected to the virtual serial port of NSLINK. If PB6 and PB7 are not used as serial ports in the project, and are used for other purposes, the serial port jumper caps must be unplugged.