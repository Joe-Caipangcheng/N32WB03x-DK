
1. Function description

    WWDG reset function.
    

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB


3. Instructions for use

    System Configuration:
       1. IWDG clock source: PCLK1(64MHZ)
	   2. Window value: 24.064 ms < n < 32.768 ms
       3. light Indicator: PB0(LED1)   PA6(LED2)

    Test steps and phenomenon：
       1. Compile and download the code to reset and run, the indicator LED2 keeps flashing. 
	      It means that the window value is refreshed normally and the code is running normally.
       2. When the parameter of the Delay() function is changed from 28 to less than 24 or greater than 33, 
	      the entire system will always be in the reset state. LED1 is on.

		
4. Matters needing attention
    1. When the window value is very small, the system is in a frequent reset state, and at this time, it is easy to 
	   cause the program to fail to download normally. In this routine, 1s delay is added before WWDG is 
	   turned on to avoid this phenomenon. Of course, without delay, you can directly pull up the BOOT0 pin to download normally.
