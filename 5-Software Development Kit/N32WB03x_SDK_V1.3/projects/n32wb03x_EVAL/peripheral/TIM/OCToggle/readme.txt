
1. Function description
    1. After TIM3 CH1 CH2 CH3 CH4 reaches the CC value, the output is flipped, and the comparison value is accumulated
    
2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for Use
    System configuration;
        1. Clock source:
            HSI=64M,AHB=64M,APB1=32M,APB2=64M, TIM3 CLK=64M
        2. Interrupt:
                    TIM3 compare interrupt on, priority 0
        3. Port configuration:
                    PB4 is selected as the CH1 output of TIM3
                    PB5 is selected as the CH2 output of TIM3
                    PB6 is selected as the CH3 output of TIM3
                    PB7 is selected as the CH4 output of TIM3
        4. TIM:
                    TIM3 configures the comparison value output of CH1 CH2 CH3 CH4 to flip, and open the comparison interrupt
    Instructions:
        1. Open the debug mode after compiling, and observe the waveforms of CH1 CH2 CH3 CH4 of TIM3 with an oscilloscope or logic analyzer
        2. Whenever the comparison value is reached, the output is flipped, and the same comparison value is increased again, and the waveform duty cycle is 50%
4. Matters needing attention
    By default, the PB6 and PB7 jumper caps of the development board are connected to the virtual serial port of NSLINK. If PB6 and PB7 are not used as serial ports in the project, and are used for other purposes, the serial port jumper caps must be unplugged.