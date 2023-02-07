
1. Function description

	/* Briefly describe the project function */
         This example configures and demonstrates the MCU benchmark


2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB
        

3. Instructions for use

	/* Describe related module configuration methods; for example: clock, I/O, etc. */
	SystemClock: HSI：64MHz
    Log print: DEMO board PB6(TX), baud rate: 115200, 8 data bits, 1 stop bit, no parity bit, no hardware flow control
	SysTick: Clock ticks configured as 1MS

	/* Describe the test steps and phenomena of the Demo */
    1. After compiling, download the program to reset and run;
    2. Print out the running score results through the serial port;


4. Matters needing attention
