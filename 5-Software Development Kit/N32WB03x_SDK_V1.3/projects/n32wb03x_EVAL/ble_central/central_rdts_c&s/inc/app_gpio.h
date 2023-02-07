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
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAYS OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file app_gpio.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __APP_GPIO_H__
#define __APP_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32wb03x.h"
    
#define LED1_PORT GPIOB
#define LED1_PIN  GPIO_PIN_0
#define LED2_PORT GPIOA
#define LED2_PIN  GPIO_PIN_6


#define KEY2_INPUT_PORT        GPIOB
#define KEY2_INPUT_PIN         GPIO_PIN_2
#define KEY2_INPUT_EXTI_LINE   EXTI_LINE2
#define KEY2_INPUT_PORT_SOURCE GPIOB_PORT_SOURCE
#define KEY2_INPUT_PIN_SOURCE  GPIO_PIN_SOURCE2
#define KEY2_INPUT_IRQn        EXTI2_3_IRQn


void LedInit(GPIO_Module* GPIOx, uint16_t Pin);
void LedOn(GPIO_Module* GPIOx, uint16_t Pin);
void LedOff(GPIO_Module* GPIOx, uint16_t Pin);
void LedBlink(GPIO_Module* GPIOx, uint16_t Pin);

void app_key_configuration(void);
void app_key_reinit_after_sleep(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_GPIO_H__ */
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
