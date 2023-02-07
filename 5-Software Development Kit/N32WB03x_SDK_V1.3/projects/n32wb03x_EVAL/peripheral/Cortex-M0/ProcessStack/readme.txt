
1. Function description

    /* Briefly describe the project function */
    Demonstrates how to modify thread mode privileged access and stack


2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB


3. Instructions for use

    /* Describe related module configuration methods; for example: clock, I/O, etc. */
        SystemClock: HSI：64MHz
        Log print: DEMO board PB6(TX), baud rate: 115200, 8 data bits, 1 stop bit, no parity bit, no hardware flow control

    /* Describe the test steps and phenomena of the Demo */
         1. After compiling, download the program to reset and run;
         2. You can use the "Cortex Register" window of the compiler to view the stack used in thread mode and the access level of the code (privileged level/unprivileged level);


4. Matters needing attention