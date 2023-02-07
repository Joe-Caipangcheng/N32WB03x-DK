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
 * @file ns_log_lpuart.c
 * @author Nations Firmware Team
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** @addtogroup 
 * @{
 */

 /* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "n32wb03x.h"
#include "ns_log.h"
#if (NS_LOG_LPUART_ENABLE)
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define _LPUART1_COM_    (0)

#if (_LPUART1_COM_ == 0)
#define LPUARTx             LPUART1
#define LPUARTx_CLK         RCC_LPUART1CLK
#define LPUARTx_GPIO        GPIOB
#define LPUARTx_GPIO_CLK    RCC_APB2_PERIPH_GPIOB
#define LPUARTx_RxPin       GPIO_PIN_2
#define LPUARTx_TxPin       GPIO_PIN_1
#define LPUARTx_Rx_GPIO_AF  GPIO_AF4_LPUART1
#define LPUARTx_Tx_GPIO_AF  GPIO_AF4_LPUART1
#else
#define LPUARTx             LPUART1
#define LPUARTx_CLK         RCC_LPUART1CLK
#define LPUARTx_GPIO        GPIOB
#define LPUARTx_GPIO_CLK    RCC_APB2_PERIPH_GPIOB
#define LPUARTx_RxPin       GPIO_PIN_11
#define LPUARTx_TxPin       GPIO_PIN_12
#define LPUARTx_Rx_GPIO_AF  GPIO_AF2_LPUART1
#define LPUARTx_Tx_GPIO_AF  GPIO_AF2_LPUART1
#endif


/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
//extern void lpuart_init(void);
void RCC_Configuration(uint32_t LPUART_CLK_SRC);
void GPIO_Configuration(void);
/* Private functions ---------------------------------------------------------*/



/**
 * @brief  init the log feature 
 */
void ns_log_lpuart_init(void)
{
    LPUART_InitType LPUART_InitStructure;
    /* System Clocks Configuration */
#if 0    
    RCC_Configuration(RCC_LPUARTCLK_SRC_LSI_LSE);
#else
    RCC_Configuration(RCC_LPUARTCLK_SRC_APB1);
#endif
    
    /* Configure the GPIO ports */
    GPIO_Configuration();

    LPUART_DeInit(LPUARTx);
    /* LPUARTx configuration ------------------------------------------------------*/
    LPUART_InitStructure.BaudRate            = 115200;
    LPUART_InitStructure.Parity              = LPUART_PE_NO;
    LPUART_InitStructure.RtsThreshold        = LPUART_RTSTH_FIFOFU;
    LPUART_InitStructure.HardwareFlowControl = LPUART_HFCTRL_NONE;
    LPUART_InitStructure.Mode                = LPUART_MODE_TX;
    /* Configure LPUARTx */
    LPUART_Init(LPUARTx, &LPUART_InitStructure);

}

/**
 * @brief  Configures the different system clocks.
 * @param  LPUART_CLK_SRC: specifies the LPUARTx clock source.
 */
void RCC_Configuration(uint32_t LPUART_CLK_SRC)
{
    switch(LPUART_CLK_SRC)
    {
        case RCC_LPUARTCLK_SRC_LSI_LSE:
        {  
            /* Specifies the LPUARTx clock source, LSE selected as LPUARTx clock */
            RCC_ConfigLpuartClk(LPUARTx_CLK, RCC_LPUARTCLK_SRC_LSI_LSE);
        }
        break;
        default:
        {
            /* Specifies the LPUARTx clock source, APB1 selected as LPUARTx clock */
            RCC_ConfigLpuartClk(LPUARTx_CLK, RCC_LPUARTCLK_SRC_APB1);
        }
        break;
    }    
    
    /* Enable LPUARTx Clock */
    RCC_EnableLpuartClk(ENABLE);    
}

/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;
    
    /* Enable GPIO clock */
    RCC_EnableAPB2PeriphClk(LPUARTx_GPIO_CLK | RCC_APB2_PERIPH_AFIO, ENABLE);
    
    /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);  
    /* Configure LPUARTx Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = LPUARTx_TxPin;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = LPUARTx_Tx_GPIO_AF;
    GPIO_InitPeripheral(LPUARTx_GPIO, &GPIO_InitStructure);

}

void ns_log_lpuart_deinit(void)
{

}

/* retarget the C library printf function to the LPUARTx */
int fputc(int ch, FILE* f)
{
    static uint8_t func_lock = 1;
    if(func_lock)
    {
        func_lock = 0;
        LPUART_SendData(LPUARTx, (uint8_t)ch);
        while (LPUART_GetFlagStatus(LPUARTx, LPUART_FLAG_TXC) == RESET);
        LPUART_ClrFlag(LPUARTx, LPUART_FLAG_TXC); 
        func_lock = 1;
    }        
    
    return (ch);
}
#endif
/**
 * @}
 */

