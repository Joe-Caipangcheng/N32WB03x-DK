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
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "main.h"
#include <stdio.h>
#include "log.h"
/**
 *  Cortex-M0 SysTick
 */


#define SYSTICK_1MS           ((uint32_t)1000)
#define DEMO_USART_BAUDRATE    ((uint32_t)115200)

uint32_t Tick_num = 0;

void rcc_debug(void)
{
    GPIO_InitType GPIO_InitStructure;
    
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_AFIO, ENABLE);
    
    GPIO_InitStruct(&GPIO_InitStructure);
    
    /* Configure rcc_mco pin*/
    GPIO_InitStructure.Pin               = GPIO_PIN_5;
    GPIO_InitStructure.GPIO_Mode         = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Speed        = GPIO_SPEED_HIGH;
    GPIO_InitStructure.GPIO_Alternate    = GPIO_AF4_MCO;
    GPIO_InitStructure.GPIO_Pull         = GPIO_NO_PULL;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
    
    RCC_ConfigMco(RCC_MCO_SYSCLK);
    printf("RCC->CFG:0x%02x\r\n", RCC->CFG);
}

/**
 * @brief  Main program.
 */
int main(void)
{
    /* USART Init */
    log_init();
    printf("Cortex-M0 SysTick \r\n");
    
    rcc_debug();
    /* Get SystemCoreClock */
    SystemCoreClockUpdate();
    printf("SystemCoreClock:%d\r\n", SystemCoreClock);
    /* Config 1s SysTick  */
    SysTick_Config(SystemCoreClock/SYSTICK_1MS);

    while (1)
    {
        if((Tick_num%1000) == 0)
        {
            printf("Cortex-M0 SysTick IRQ \r\n");
        }
    }
}

/**
 * @}
 */

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
*          line: assert_param error line source number
 * @return None
 */
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {}
}

/**
 * @}
 */
#endif
