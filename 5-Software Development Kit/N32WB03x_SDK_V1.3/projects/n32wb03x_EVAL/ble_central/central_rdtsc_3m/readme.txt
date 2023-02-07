
1. Function description

    Bluetooth BLE data transmission service (rdts) master example, using 128bit UUID. 
    After connecting the slave device, the data received by Bluetooth will be transparently 
    transmitted to USART1, and the data received by USART1 will be transparently transmitted 
    to the slave device.
    This example is allow connect up to 3 slave deivce(rdtss example). It needs to set the 
    active connection befor sending data. The first byte will be the connection to send if 
    it in 0x00 to 0x02

2. Operating environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions

    System configuration;
        1. Clock source: HSI=64M, AHB=64M, APB1=32M, APB2=64M
        2. Port configuration:
            LED1: PB0
            LED2: PA6
            KEY1: PB1
            KEY2: PB2
        3. Log print: DEMO board PB1(TX), baud rate: 115200, 8 data bits, 1 stop bit, no parity bit, no hardware flow control
        4. BLE Data: USART1 PB6(TX) and PB7(RX), baud rate: 115200, 8 data bits, 1 stop bit, no parity bit, no hardware flow control
        5. LED1 lights up when it is turned on, LED2 indicates the connection status, and lights up when Bluetooth is connected
        6. The counter test routine is the rdtss slave routine in the ble directory
        
    usage method:
        1. Burn to development board after compilation
        2. Power on development board and check the log.
        3. Connect the salve device, and send data via USART1 each other.

4. Precautions
    Since the program will enter the sleep mode, the SWD will not be accessible. 
    Press the RESET button to execute the program burning step within one second.
