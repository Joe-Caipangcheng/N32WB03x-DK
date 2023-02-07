1. Function description

    Bluetooth BLE heart rate service routine. After the BLE host connects to the device, 
    the chip automatically generates heart rate data and reports it through the Bluetooth heart rate service.

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
        3. Log print: DEMO board PB6(TX), baud rate: 115200, 8 data bits, 1 stop bit, no parity bit, no hardware flow control
        4. Bluetooth BLE name: NS_Heart_Rate
        5. LED1 lights up when it is turned on, LED2 indicates the connection status, and lights up when Bluetooth is connected
        6. After Bluetooth connection, the heart rate data will be reported, and the heart rate data will increase from 35 to 180,
         so as to cycle.   

    usage method:
        1. Burn to development board after compilation
        2. Power on development board and check the log.
        3. Connect the BLE device with the name above, and check the BLE data upstreaming.

4. Precautions
    Since the program will enter the sleep mode, the SWD will not be accessible. Press the RESET button to execute the program burning step within one second.