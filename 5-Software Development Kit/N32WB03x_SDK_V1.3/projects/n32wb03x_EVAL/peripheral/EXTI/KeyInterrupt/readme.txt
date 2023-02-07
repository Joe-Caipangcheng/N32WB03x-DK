
1. Function description

    This example shows how to control LED blinking by externally triggering an interrupt.
    
2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB


3. Instructions for use

     //Describe related module configuration methods; for example, clock, I/O, etc.
     1. SystemClock: HSI：64MHz
     2. GPIO: PB2 is selected as the external interrupt entry, PA6 controls the LED2 to flash.

     //Describe the test steps and phenomena of the Demo
     1. Compile and download the code to reset and run.
     2. Press and release the KEY2 button,then the LED will flash.

4. Matters needing attention
     None