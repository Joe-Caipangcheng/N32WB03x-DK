
1. Function description

     This example demonstrates the communication with the external EEPRON through the I2C module.

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB


3. Instructions for use

     1. SystemClock: HSI 64MHz
     2. I2C1 configuration:
             SCL --> PB7
             SDA --> PB6
             CLOCK: 100KHz
            
     3. USART2 configuration:
             TX --> PB4
             RX --> PB5
             Baud rate: 115200
    
     4. Test using EEPROM model:
            AT24C04 (capacity 4kb)
        

     5. Test steps and phenomena
         a, check the EEPROM connection
         b, compile and download the code, reset and run
         c, view the print information from the serial port and verify the result

4. Matters needing attention
    SCL and SDA must be pulled up