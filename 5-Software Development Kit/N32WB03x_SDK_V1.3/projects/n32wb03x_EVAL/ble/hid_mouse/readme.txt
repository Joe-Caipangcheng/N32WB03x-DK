
1. Function description

    Bluetooth BLE HID mouse service routine. After the BLE host is connected to the device, 
    the mouse displacement command is triggered through button1, and the multimedia volume 
    decrement command is triggered through button2.

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
        4. Bluetooth BLE name: hid_mouse
        5. LED1 lights up when it is turned on, LED2 indicates the connection status, and lights up when Bluetooth is connected


    usage method:
        1. Burn to development board after compilation
        2. Power on development board and check the log.
        3. Search and connect devices on the Bluetooth page of the system in smartphone. The pairing password is 123456.
        4. Press button1 to send the mouse displacement command, and the mouse moves 10 pixels to the right.
        5. Press button2 to send multimedia key value, and the volume decreases.

4. Precautions
    Since the program will enter the sleep mode, the SWD will not be accessible. Press the RESET button to execute the program burning step within one second.