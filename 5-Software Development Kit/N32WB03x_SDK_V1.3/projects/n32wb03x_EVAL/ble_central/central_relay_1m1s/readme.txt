
1. Function description
    Bluetooth BLE realy example With 128bit UUID, the relay device can be connected to one master and 
    one slave at the same time. After establishment, the received slave data can be sent to the master, 
    received master data can be sent to the slave.

2. Operating environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions
    System configuration;
    1. Clock source: HSI=64M
    2. LED1/LED2 indicates the connection status, and lights up when Bluetooth is connected.

        Test steps:
        1. Prepare two N32WB031_ STB_ V1.1 Development board and a mobile phone.
        2. Connect SWD and USART1 on development board J10.
        3. Connect development boards J7 and J8 to supply power to MCU and NS Link.
        4. Download a master slave relay routine to one development board (relay development board). 
            The project path is: ble_ central\central_ relay_ 1m1s\MDK-ARM\central_ relay.uvprojx.

        The following "5" or "6" methods can be used to establish the connection between the master 
        and slave at the same time:

5. Download slave routines to another development board (slave development board). 
    Project path: \ble\rtdss\MDK-ARM\rdtss.uvprojx.
    Use the default Bluetooth MAC address: " x0C x02 x03 x04 x05 x06"

    5.1 After the two development boards are powered on, the relay development board automatically 
        connects to the slave development board. When the relay development board LED1 lights up, it 
        indicates that the connection with the slave is successful.
    5.2 Use APP to scan BLE devices and connect devices "NS_ RDTS_ RELAY ", when the relay development 
        board LED2 lights up, it indicates that the connection with the host is successful.

6. Download slave routines to another development board (slave development board). Project path:
    N32WB03x_ SDK-realy\projects\n32wb03x_ EVAL\ble\rtdss\MDK-ARM\rdtss.uvprojx.
    Before downloading, modify the Bluetooth MAC address to: " x0C x02 x03 x04 x05 x07"
    6.1 Use APP to scan BLE devices and connect devices "NS_ RDTS_ RELAY ", when the relay development board 
        LED2 lights up, it indicates that the connection with the host is successful.
    6.2 Use the APP to scan the BLE device to ensure that the device with the address 
        of " x0C x02 x03 x04 x05 x07" can be scanned.
    6.3 Enable NS on mobile APP_ RDTS_ Notify permission of the second notify feature of RELAY.
    6.4 On the mobile phone APP, turn to NS_ RDTS_ Write data in the second write feature of RELAY: 01000C0203040507
    6.5 When the relay development board LED1 lights up, it indicates that the connection with the slave is successful.

    7. Enable NS on mobile APP_ RDTS_ Notify permission of the first notify feature of RELAY.
    8. Send data to the first write feature of the relay development board through the mobile phone APP,
        and the corresponding data can be received at the serial port of the slave development board.
    9. After sending data to the slave development board through the serial port, the first notify feature 
        of the phone can receive the data.

4. Process description:
    After the relay device is powered on,  the device is scanning and broadcasting at the same time.
    When the relay is connected to the master device, the broadcast stops; When the relay is disconnected 
    from the master device, the broadcast starts.
    When the relay is connected to the peer slave device, the scanning stops; When the relay is disconnected 
    from the peer slave device, the scan starts.

5. Description of relay as a slave profile
    Service: 0x01102EC78a0E7390E111C20860270000
    RDTSS_ RX: 0x01002EC78a0E7390E111C20860270000(Write) mobile phone send data to this UUID will be relayed 
        to the slave device connected to the relay device
    RDTSS_ TX: 0x02002EC78a0E7390E111C20860270000(NTF) After the slave device connected to the relay device 
        sends the notify data, the data is relayed to the mobile phone through this feature
    Control_ Point: 0x03002EC78a0E7390E111C20860270000(Write) The mobile phone can send control commands to relay 
        through this feature (for example, connect the slave at a specific address and disconnect the slave)
    Command_ Complete: 0x04002EC78a0E7390E111C20860270000(NTF) Mobile phone returns Command completion status, 
        when the relay is connected/disconnected from the slave, report the status to the mobile phone.

6、Control_ Point Command
    Command data structure:
        value[0] = cmd;
        value[1...] = data;
    Cmd=01 Connect slave
    Example of a complete command: 01000C0203040507
        01: Connect slave
        00: Slave address type: Public
        0C0203040507: Slave address
    Cmd=02 Disconnect from slave
    Example of a complete command: 02
        02: Disconnect the slave

7、Command_ Complete event
    Event data structure:
    value[0] = CNTL_ POINT_ CMD_ CMP;
    value[1] = cmd;
    value[2] = status;
    value[3] = length;
    value[4...] = data;

5. Precautions
    Since the program will enter the sleep mode, the SWD will not be accessible. Press the RESET button 
    to execute the program burning step within one second. 