
1. Function description
    This routine provides a DMA usage for transferring data between I2C peripheral and RAM.
    This demo as I2C master device, it need another board as slave device to demonstrates, we can use I2C/I2C_Slave example.

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions for use
    1.Clock source: HSI 64M
    2.DMA channels: DMA_CH4, DMA_CH5
    3.I2C1 configuration:
        SCL   -->  PB6
        SDA   -->  PB7
        ADDR:0x30(7bit)
        CLOCK:100K

4.Log print: DEMO board PB4(TX), baud rate: 115200, 8 data bits, 1 stop bit, no parity bit, no hardware flow control

5.Test steps and phenomena
    1. Compile download code reset run
    2. View the printed information from the serial port and verify the result

6. Precautions
    None