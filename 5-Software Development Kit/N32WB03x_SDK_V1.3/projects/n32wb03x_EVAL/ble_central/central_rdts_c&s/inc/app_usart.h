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
 * @file app_usart.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __APP_USART_H__
#define __APP_USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32wb03x.h"

    
/* Public typedef -----------------------------------------------------------*/
/* Public define ------------------------------------------------------------*/
#define _USART1_COM_
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

#define USARTx_DAT_Base         (USART1_BASE + 0x04)
#define USARTx_Tx_DMA_Channel   DMA_CH1
#define USARTx_Tx_DMA_FLAG      DMA_FLAG_TC1
#define USARTx_Rx_DMA_Channel   DMA_CH2
#define USARTx_Rx_DMA_FLAG      DMA_FLAG_TC2
#define USARTx_Tx_DMA_REMAP     DMA_REMAP_USART1_TX
#define USARTx_Rx_DMA_REMAP     DMA_REMAP_USART1_RX
#define USARTx_IRQn             USART1_IRQn
#define USARTx_IRQHandler       USART1_IRQHandler
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

#define USARTx_DAT_Base         (USART2_BASE + 0x04)
#define USARTx_Tx_DMA_Channel   DMA_CH1
#define USARTx_Tx_DMA_FLAG      DMA_FLAG_TC1 
#define USARTx_Rx_DMA_Channel   DMA_CH2
#define USARTx_Rx_DMA_FLAG      DMA_FLAG_TC2
#define USARTx_Tx_DMA_REMAP     DMA_REMAP_USART2_TX
#define USARTx_Rx_DMA_REMAP     DMA_REMAP_USART2_RX
#define USARTx_IRQn             USART2_IRQn
#define USARTx_IRQHandler       USART2_IRQHandler
#endif


#define USART_RX_DMA_SIZE  256
#define USART_RX_FIFO_SIZE 1000
#define USART_TX_FIFO_SIZE 1000


/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/

/* Public function prototypes -----------------------------------------------*/

void app_usart_configuration(void);
uint8_t app_usart_rx_data_fifo_enter(const uint8_t *p_data, uint16_t len);
uint8_t usart_tx_dma_send(uint8_t *p_data, uint16_t len);
void app_usart_tx_data_blocking(uint8_t *p_data, uint16_t len);

uint8_t app_usart_tx_fifo_enter(const uint8_t *p_data, uint16_t len);
void usart_forward_to_ble_loop(void);
void app_usart_dma_enable(FunctionalState Cmd);
void app_usart_tx_process(void);
#ifdef __cplusplus
}
#endif

#endif /* __APP_USART_H__ */
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
