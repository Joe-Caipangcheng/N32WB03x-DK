
1. Function description

    Bluetooth BLE data transmission service (rdts) master slave switching routine, using 128bit UUID. 
    After connecting the slave/host device, the data received by Bluetooth will be transparently transmitted to USART1, 
    and the data received by USART1 will be transparently transmitted to the slave device.
    Press button2 to switch the master-slave mode. In addition, the serial port can also send two bytes of 0xfa 0x55.
    Note that since the serial port takes effect only after connection, the switch command can only be sent after connection.
    Firmware pass through variable fw_ Mode control mode. The default is slave mode.

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
        4. Bluetooth BLE name: NS_RDTSS
        5. LED1 lights up when it is turned on, LED2 indicates the connection status, and lights up when Bluetooth is connected
        6. BLE Data: USART1 PB6(TX) and PB7(RX), baud rate: 115200, 8 data bits, 1 stop bit, no parity bit, no hardware flow control
        7. This example can test with \ble\rtdss or ble_central\central_rdtsc.
        8. In any state, press Button2 to switch between master and slave modes.
        9. Or when the connection is maintained, the serial port sends two bytes 0xfa 0x55 to switch the master slave mode.

    usage method:
        1. Burn to development board after compilation
        2. Power on development board and check the log.
        3. Set up another development board to as master(or slave) to connected.
        4. Send data via USART1 each other.

4. Precautions
    Since the program will enter the sleep mode, the SWD will not be accessible. Press the RESET button to execute the program burning step within one second.
