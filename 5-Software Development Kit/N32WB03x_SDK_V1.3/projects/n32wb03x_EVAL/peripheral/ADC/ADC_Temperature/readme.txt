
1. Function description
    1. ADC samples and converts the analog voltage of the internal temperature sensor and converts it to a temperature value

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for use
    System Configuration;
        1. Clock source:
                    HSE=32M, HSI=64M, AHB=64M, APB1=32M, APB2=64M, ADC CLK=HSE/8
        2. Port configuration:
                    PB8 is selected as analog function, ADC conversion channel 3
        4. ADC:
                    ADC configuration: continuous conversion, software trigger, 10-bit data, conversion channel 7 is the analog voltage data of the internal temperature sensor
        5. Log print: DEMO board PB6(TX), baud rate: 115200, 8 data bits, 1 stop bit, no parity bit, no hardware flow control
    Instructions:
        1. Download after compiling and reset to rum the program, check conversion result at log.
        2. The serial port tool displays the temperature value in the real-time chip

4. Matters needing attention
    Please make sure connect the J15 to left side and J16 to right side which is connect the IO to pogo pin not the crystal.