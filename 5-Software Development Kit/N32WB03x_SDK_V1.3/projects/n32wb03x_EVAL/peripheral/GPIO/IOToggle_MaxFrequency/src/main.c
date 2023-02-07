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
#include <stdint.h>

GPIO_InitType GPIO_InitStructure;

/**
 * @brief  Main program.
 */
int main(void)
{
    /* -1- Enable GPIOx Clock (to be able to program the configuration registers) */
    RCC_EnableAPB2PeriphClk(GPIOx_CLK, ENABLE);

    /* -2- Configure GPIOx_PIN in output push-pull mode */
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIOx_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_HIGH;
    GPIO_InitPeripheral(GPIOx, &GPIO_InitStructure);

    /* -3- Toggle GPIOx_PIN in an infinite loop */
    while (1)
    {          
        GPIOA->POD = 1;
        GPIOA->POD = 0;
        GPIOA->POD = 1;
        GPIOA->POD = 0;
        GPIOA->POD = 1;
        GPIOA->POD = 0;
        GPIOA->POD = 1;
        GPIOA->POD = 0;
        GPIOA->POD = 1;
        GPIOA->POD = 0;
        GPIOA->POD = 1;
        GPIOA->POD = 0;
        GPIOA->POD = 1;
        GPIOA->POD = 0;
        GPIOA->POD = 1;
        GPIOA->POD = 0;
        GPIOA->POD = 1;
        GPIOA->POD = 0;
        GPIOA->POD = 1;
        GPIOA->POD = 0;
    }
}

/**
 * @brief Assert failed function by user.
 * @param file The name of the call that failed.
 * @param line The source line number of the call that failed.
 */
#ifdef USE_FULL_ASSERT
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    while (1)
    {
    }
}
#endif // USE_FULL_ASSERT
/**
 * @}
 */
