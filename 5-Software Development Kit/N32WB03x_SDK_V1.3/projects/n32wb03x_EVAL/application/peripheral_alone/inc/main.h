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
 * @file main.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include "n32wb03x.h"


    
typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} Status;
#define GLOBAL_INT_DISABLE()         \
uint32_t ui32IntStatus = 0;          \
do{                                  \
    ui32IntStatus = __get_PRIMASK(); \
    __set_PRIMASK(1);                \
}while(0)

#define GLOBAL_INT_RESTORE()      \
do{                               \
    __set_PRIMASK(ui32IntStatus); \
}while(0)

#define _USART1_COM_  //don't use it with ADC test at the same time, PB7 as ADC CH4. 
//#define _USART2_COM_

#ifdef _USART1_COM_
#define USARTx              USART1
#define USARTx_CLK          RCC_APB2_PERIPH_USART1
#define USARTx_GPIO         GPIOB
#define USARTx_GPIO_CLK     RCC_APB2_PERIPH_GPIOB
#define USARTx_RxPin        GPIO_PIN_7
#define USARTx_TxPin        GPIO_PIN_6
#define USARTx_Rx_GPIO_AF   GPIO_AF4_USART1
#define USARTx_Tx_GPIO_AF   GPIO_AF4_USART1
#define GPIO_APBxClkCmd     RCC_EnableAPB2PeriphClk
#define USART_APBxClkCmd    RCC_EnableAPB2PeriphClk
#endif

#ifdef _USART2_COM_
#define USARTx              USART2
#define USARTx_CLK          RCC_APB1_PERIPH_USART2
#define USARTx_GPIO         GPIOB
#define USARTx_GPIO_CLK     RCC_APB2_PERIPH_GPIOB
#define USARTx_RxPin        GPIO_PIN_5
#define USARTx_TxPin        GPIO_PIN_4
#define USARTx_Rx_GPIO_AF   GPIO_AF3_USART2
#define USARTx_Tx_GPIO_AF   GPIO_AF3_USART2
#define GPIO_APBxClkCmd     RCC_EnableAPB2PeriphClk
#define USART_APBxClkCmd    RCC_EnableAPB1PeriphClk
#endif


#define LED1_PORT GPIOB
#define LED1_PIN  GPIO_PIN_0
#define LED2_PORT GPIOA
#define LED2_PIN  GPIO_PIN_6

#define KEY1_INPUT_PORT        GPIOB
#define KEY1_INPUT_PIN         GPIO_PIN_1
#define KEY1_INPUT_EXTI_LINE   EXTI_LINE1
#define KEY1_INPUT_PORT_SOURCE GPIOB_PORT_SOURCE
#define KEY1_INPUT_PIN_SOURCE  GPIO_PIN_SOURCE1
#define KEY1_INPUT_IRQn        EXTI0_1_IRQn

#define KEY2_INPUT_PORT        GPIOB
#define KEY2_INPUT_PIN         GPIO_PIN_2
#define KEY2_INPUT_EXTI_LINE   EXTI_LINE2
#define KEY2_INPUT_PORT_SOURCE GPIOB_PORT_SOURCE
#define KEY2_INPUT_PIN_SOURCE  GPIO_PIN_SOURCE2
#define KEY2_INPUT_IRQn        EXTI2_3_IRQn



void LedBlink(GPIO_Module* GPIOx, uint16_t Pin);
void LedOn(GPIO_Module* GPIOx, uint16_t Pin);
void LedOff(GPIO_Module* GPIOx, uint16_t Pin);
uint16_t ADC_GetData_blocking(uint32_t ADC_Channel);
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
