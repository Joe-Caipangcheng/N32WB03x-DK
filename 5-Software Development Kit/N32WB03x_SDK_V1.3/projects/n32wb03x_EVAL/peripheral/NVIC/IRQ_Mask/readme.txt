
1. Function description

    /* Briefly describe the project function */
    This routine configures and demonstrates the use of EXIT external interrupt and TIM timer interrupt

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB
        

3. Instructions for use

    /* Describe related module configuration methods; for example: clock, I/O, etc. */
    SystemClock: HSI 64MHz
    USART: TX - PB6, baud rate 115200
    EXIT: PB1 is floating input mode, external interrupt line - EXIT_LINE1, open external interrupt
    TIM: prescale factor - (SystemClock/1200-1), period - (1200-1), enable timer interrupt

    /* Describe the test steps and phenomena of the Demo */
     1. After compiling, download the program to reset and run;
     2. Check the serial port printing information, the timer interrupt information is printed every 1S, press the button to stop printing, press it again to continue printing, indicating that the program is running normally;


4. Matters needing attention
