1、功能说明
    1、LED1 和 LED2 启动即亮起
    2、短按button1可以进入sleep，长按button1进入sleep模式和RTC唤醒功能，sleep可以通过短按button唤醒
    3、短按button2读取 ADC CH1（PB10)的值并输出到串口
    4、定时器TIM3 CH1（PB4） CH2（PB5）输出频率相同占空比不同的PWM
    5、串口以115200 8n1的参数运行回环应用，输入小于256字节的字串将即时返回输出
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.24.2.0
    硬件环境：      基于N32WB031_STB开发板
3、使用说明
    系统配置；
        1、时钟源：HSI=64M,AHB=64M,APB1=32M,APB2=64M
        2、端口配置：
                    PB4选择为TIM3的CH1输出
                    PB5选择为TIM3的CH2输出
                    PB6选择为USART的TX
                    PB7选择为USART的RX
                    PB10选择为ADC的CH1
                    PB0选择为LED1 
                    PA6选择为LED2 
                    PB1选择为KEY1
                    PB2选择为KEY2
        3、TIM3 的频率为40KHz, CH1占空比为75%，CH2占空比为50%
        4、串口使用的是USART1，波特率是115200，8bit数据位，无校验位，1bit停止位，无流控。
        5、ADC CH1可以测量的范围是0.15～0.95V
        
    
    使用方法：
        1、编译后烧录到开发板
        2、用示波器或者逻辑分析仪观察PB4,PB5的波形
        3、连接串口工具到USART1
        4、程序运行后，板上LED1和LED2亮起
        6、发送任意字串（低于256字节）到串口即可收到返回一样的字串
        6、短按button1将进入睡眠，按button1或者button2可以唤醒
        7、长按button1将进入睡眠，并启动RTC的唤醒功能，10秒后将唤醒MCU。
        8、短按button2将翻转LED1和 读取ADC值并转为电压值（mA）。
4、注意事项
    1、如果板子上无HSE晶体，请在option C/C++页定义宏NO_HSE_ON_BOARD，启用直接写HSI trim值，默认值为0x4F。
       如频率明显不对可以自行微调，后续将在生产中把正确trim值写入flash，用户只需读取和写入寄存器即可。
    2、MCU睡眠期间不能烧录程序，请复位后烧录。 


1. Function description
    1. LED1 and LED2 light up upon startup
    2. Short press button1 to enter the sleep mode, long press button1 to enter the sleep mode and
     RTC wake-up function, and short press button1 to wake up the sleep
    3. Short press button2 to read the value of ADC CH1 (PB10) and output it to the serial port
    4. Timer TIM3 CH1 (PB4) CH2 (PB5) PWM with the same output frequency and different duty cycle
    5. The serial port runs the loopback application with 115200 8n1 parameters, and the input 
    string of less than 256 bytes will return to the output immediately

2. Operating environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB

3. Instructions
    System configuration;
        1. Clock source: HSI=64M, AHB=64M, APB1=32M, APB2=64M
        2. Port configuration:
            PB4 selects CH1 output of TIM3
            PB5 selects CH2 output of TIM3
            PB6 Select TX as USART
            PB7 RX selected as USART
            PB10 is selected as CH1 of ADC
            Select LED1 for PB0
            Select LED2 for PA6
            Select KEY1 for PB1
            Select KEY2 for PB2
        3. The frequency of TIM3 is 40KHz, the duty cycle of CH1 is 75%, and the duty cycle of CH2 is 50%
        4. Serial port uses USART1, the baud rate is 115200, 8bit data bit, no check bit, 1bit stop bit, no flow control.
        5. ADC CH1 can measure from 0.15 to 0.95V

    usage method:
        1. Burn to development board after compilation
        2. Observe the waveform of PB4 and PB5 with an oscilloscope or logic analyzer
        3. Connect the serial port tool to USART1
        4. After the program runs, LED1 and LED2 on the board light up
        6. Send any string (less than 256 bytes) to the serial port to receive and return the same string
        6. Short press button1 to go to sleep, and press button1 or button2 to wake up
        7. Long press button1 to enter sleep, and start the wake-up function of RTC. After 10 seconds, MCU will wake up.
        8. Short press button 2 to flip LED1, read the ADC value, and convert it to voltage (mA).

4. Precautions
    1. If there is no HSE crystal on the board, please define macro NO on the option C/C++page_ HSE_ ON_ BOORD: Enable direct
    writing of HSI to trim values. The default value is 0x4F.
        If the frequency is wrong, you can fine-tune it by yourself. Later, you will write the correct trim value to 
    flash in production, and users only need to read and write the register.
    2. Program cannot be burned during MCU sleep. Please reset and burn.