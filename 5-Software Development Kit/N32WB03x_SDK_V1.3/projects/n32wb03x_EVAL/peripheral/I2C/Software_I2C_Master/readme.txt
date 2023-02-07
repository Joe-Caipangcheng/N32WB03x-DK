
1. Function description

     This example demonstrates the use of software I2C as a master to send and receive data.

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
        

     4. Test steps and phenomena
         a, jumper wire to connect slave I2C1
         b, compile and download the code, reset and run
         c, view the print information from the serial port and verify the result

4. Matters needing attention
     SCL and SDA must be pulled up