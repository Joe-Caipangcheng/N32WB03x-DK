
1. Function description
    1. Entry and wake-up exit of PD mode.

2. Use environment
    Software development environment: KEIL MDK-ARM V5.30.0.0
    Hardware environment: Developed based on the development board N32WB031_STB


3. Instructions for use
    System Configuration;
        1. SystemClock: HSI 64MHz
        2. GPIO: PB0, PA6 control LED1, LED2

    Instructions:
        After compiling under KEIL, it is burned to the evaluation board, and the ammeter is connected in series. After a while, the current becomes significantly smaller. Press the WAKEUP button and the current returns to mA level. After a while it drops to uA again.
        The LED1(PB0) light on when in run mode, after a while it will trun of LED1 and enter PD mode, press the WAKEUP button will wake up and resrart to run, and light on LED1(PB0).
            

4. Matters needing attention
    none
