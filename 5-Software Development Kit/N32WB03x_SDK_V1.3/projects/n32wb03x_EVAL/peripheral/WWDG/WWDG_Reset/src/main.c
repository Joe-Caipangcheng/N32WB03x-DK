/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file main.c
 * @author Nations Firmware Team
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "main.h"
#include "n32wb03x.h"
#include "log.h"

/** @addtogroup n32wb03x_StdPeriph_Examples
 * @{
 */

/** @addtogroup WWDG_Reset
 * @{
 */

__IO uint32_t TimingDelay = 0;

static void Delay(__IO uint32_t nTime);

/**
 * @brief  Configures LED GPIO.
 * @param Led Specifies the Led to be configured.
 *   This parameter can be GPIO_PIN_0~GPIO_PIN_15.
 */
void LedInit(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_InitType GPIO_InitStructure;

    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

    /* Enable the GPIO Clock */
    if (GPIOx == GPIOA)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    }
    else if (GPIOx == GPIOB)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    }
    
    /* Configure the GPIO pin */
    if (Pin <= GPIO_PIN_ALL)
    {
        GPIO_InitStruct(&GPIO_InitStructure);
        GPIO_InitStructure.Pin        = Pin;
        GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_OUTPUT_PP;
        GPIO_InitPeripheral(GPIOx, &GPIO_InitStructure);
    }
}

/**
 * @brief  Turns selected Led on.
 * @param Led Specifies the Led to be set on.
 */
void LedOn(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIOx->PBSC = Pin;
}
/**
 * @brief  Turns selected Led Off.
 * @param Led Specifies the Led to be set off.
 */
void LedOff(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIOx->PBC = Pin;
}

/**
 * @brief  Toggles the selected Led.
 * @param Led Specifies the Led to be toggled.
 */
void LedBlink(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIOx->POD ^= Pin;
}

/**
 * @brief  Main program.
 */
int main(void)
{
    /*!< At this stage the microcontroller clock setting is already configured,
         this is done through SystemInit() function which is called from startup
         file (startup_n32wb03x.s) before to branch to application main.
         To reconfigure the default setting of SystemInit() function, refer to
         system_n32wb03x.c file
       */
	RCC_ClocksType RCC_Clocks;
	
	log_init();
	printf("\r\n This is WWDG Reset demo\r\n");
	
	WWDG_ClrEWINTF();
	
    LedInit(LED1_PORT, LED1_PIN);
	LedInit(LED2_PORT, LED2_PIN);
	
    LedOff(LED1_PORT, LED1_PIN);
	LedOff(LED2_PORT, LED2_PIN);
	
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	 /* Setup SysTick Timer for 1 msec interrupts  */
    if (SysTick_Config(SystemCoreClock / 1000))
    {
        /* Capture error */
        while (1)
            ;
    }
	
	/* Enable PWR Clock */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, ENABLE);
	DBG_ConfigPeriph(DBG_WWDG_STOP, ENABLE);
	
    /* Check if the system has resumed from WWDG reset */
    if (RCC_GetFlagStatus(RCC_CTRLSTS_FLAG_WWDGRSTF) != RESET)
    {
        /* WWDGRST flag set, Turn on LED2*/
        LedOn(LED1_PORT, LED1_PIN);
		log_info("reset by WWDG\n");
        /* Clear reset flags */
        RCC_ClrFlag();
    }
    else
    {
        /* WWDGRST flag is not set, Turn off LED2*/
        LedOff(LED1_PORT, LED1_PIN);
    }

    /* WWDG configuration */
    /* Enable WWDG clock */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_WWDG, ENABLE);
	
	RCC_GetClocksFreqValue(&RCC_Clocks);
	printf("Pclk1Freq:%d\r\n", RCC_Clocks.Pclk1Freq);

    /* WWDG clock counter = 4096/(PCLK1(32MHz)/4) = (~512 us)  */
    WWDG_SetPrescalerDiv(WWDG_PRESCALER_DIV4);

    /* Set Window value to 80; WWDG counter should be refreshed only when the counter
      is below 80 (and greater than 64) otherwise a reset will be generated */
    WWDG_SetWValue(80);

    /*
      Enable WWDG and set counter value to 127, WWDG timeout = ~512 us * 64 = 32.768 ms
      In this case the refresh window is: ~512 us * (127-80) = 24.064 ms < refresh window < ~512 us * (127-64-1) = 32.768ms
    */
    WWDG_Enable(127);

    while (1)
    {
        /* Toggle LED2 */
        LedBlink(LED2_PORT, LED2_PIN);
        Delay(28);
        /* Update WWDG counter */
			
        WWDG_SetCnt(127);
    }
}

/**
 * @brief  Inserts a delay time.
 * @param nTime specifies the delay time length, in milliseconds.
 */
static void Delay(__IO uint32_t nTime)
{
    TimingDelay = nTime;
    while (TimingDelay != 0)
    {
    }
}

#ifdef USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param file pointer to the source file name
 * @param line assert_param error line source number
 */
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}

#endif

/**
 * @}
 */

/**
 * @}
 */
