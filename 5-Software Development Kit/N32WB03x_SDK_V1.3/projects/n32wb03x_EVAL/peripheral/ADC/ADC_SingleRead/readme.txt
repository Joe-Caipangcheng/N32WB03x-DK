
1. Function description
    1. ADC samples and converts the analog voltage of the PB8 pin and PB10 pin.
    2. Use software to trigger once and collect once

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for use
    System Configuration;
        1. Clock source:
                    HSE=32M, HSI=64M, AHB=64M, APB1=32M, APB2=64M, ADC CLK=HSE/8
        2. Port configuration:
                    PB8 is selected as analog function, ADC conversion channel 3 (hight voltage channel)
                    PB10 is selected as analog function, ADC conversion channel 1 (low voltage channel)
        3. ADC:
                    ADC configuration: continuous conversion, software trigger, 10-bit data, conversion channel 3 is the analog voltage data of PB8,
                    conversion channel 1 is the analog voltage data of PB10
        4. Log print: DEMO board PB6(TX), baud rate: 115200, 8 data bits, 1 stop bit, no parity bit, no hardware flow control
    Instructions:
        1. Download after compiling and reset to rum the program, check conversion result at log.
        2. By changing the voltage of the PA1 pin, you can see that the conversion result variable changes synchronously

4. Matters needing attention
    Please make sure connect the J15 to left side and J16 to right side which is connect the IO to pogo pin not the crystal.