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

#include "n32wb03x.h"

#define _USART1_USART2_

#ifdef _USART1_USART2_
#define USARTy                  USART1
#define USARTy_CLK              RCC_APB2_PERIPH_USART1
#define USARTy_GPIO             GPIOB
#define USARTy_GPIO_CLK         RCC_APB2_PERIPH_GPIOB
#define USARTy_RxPin            GPIO_PIN_7
#define USARTy_TxPin            GPIO_PIN_6
#define USARTy_Rx_GPIO_AF       GPIO_AF4_USART1
#define USARTy_Tx_GPIO_AF       GPIO_AF4_USART1
#define USARTy_APBxClkCmd       RCC_EnableAPB2PeriphClk
#define USARTy_DAT_Base         (USART1_BASE + 0x04)
#define USARTy_Tx_DMA_Channel   DMA_CH1
#define USARTy_Tx_DMA_FLAG      DMA_FLAG_TC1
#define USARTy_Tx_DMA_REMAP     DMA_REMAP_USART1_TX


#define USARTz                  USART2
#define USARTz_CLK              RCC_APB1_PERIPH_USART2
#define USARTz_GPIO             GPIOB
#define USARTz_GPIO_CLK         RCC_APB2_PERIPH_GPIOB
#define USARTz_RxPin            GPIO_PIN_5
#define USARTz_TxPin            GPIO_PIN_4
#define USARTz_Rx_GPIO_AF       GPIO_AF3_USART2
#define USARTz_Tx_GPIO_AF       GPIO_AF3_USART2
#define USARTz_APBxClkCmd       RCC_EnableAPB1PeriphClk
#define USARTz_DAT_Base         (USART2_BASE + 0x04)
#define USARTz_Tx_DMA_Channel   DMA_CH3
#define USARTz_Tx_DMA_FLAG      DMA_FLAG_TC3
#define USARTz_Tx_DMA_REMAP     DMA_REMAP_USART2_TX
#define USARTz_IRQn             USART2_IRQn
#endif

typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;

void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void DMA_Configuration(void);
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */
