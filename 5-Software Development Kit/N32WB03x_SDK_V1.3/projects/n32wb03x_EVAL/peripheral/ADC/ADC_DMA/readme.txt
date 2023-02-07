
1. Function description
    1. ADC samples and converts the analog voltage of the PB8 pin
    2. The ADC conversion result is read to the variable ADCConvertedValue through the DMA_CH1 channel

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for use
    System Configuration;
        1. Clock source:
                    HSE=32M, HSI=64M, AHB=64M, APB1=32M, APB2=64M, ADC CLK=HSE/8, DMA CLK=64M
        2. Port configuration:
                    PB8 is selected as analog function, ADC conversion channel 3
        3. DMA:
                    DMA1_CH1 channel is configuered as circular mode, carries a half-word ADC conversion result to ADCConvertedValue variable
        4. ADC:
                    ADC configuration: continuous conversion, software trigger, 10-bit data, conversion channel 3 is the analog voltage data of PB8
        5. Log print: DEMO board PB6(TX), baud rate: 115200, 8 data bits, 1 stop bit, no parity bit, no hardware flow control
    Instructions:
        1. Download after compiling and reset to rum the program, check conversion result at log.
        2. By changing the voltage of the PA1 pin, you can see that the conversion result variable changes synchronously

4. Matters needing attention
    Please make sure connect the J15 to left side and J16 to right side which is connect the IO to pogo pin not the crystal.