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
#include <stdio.h>
#include "main.h"

/** @addtogroup N32WB03X_StdPeriph_Examples
 * @{
 */

/** @addtogroup LPUART_Printf
 * @{
 */

LPUART_InitType LPUART_InitStructure;

/**
 * @brief  Main program
 */
int main(void)
{
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
    LPUART_InitStructure.BaudRate            = 9600;
    LPUART_InitStructure.Parity              = LPUART_PE_NO;
    LPUART_InitStructure.RtsThreshold        = LPUART_RTSTH_FIFOFU;
    LPUART_InitStructure.HardwareFlowControl = LPUART_HFCTRL_NONE;
    LPUART_InitStructure.Mode                = LPUART_MODE_RX | LPUART_MODE_TX;
    /* Configure LPUARTx */
    LPUART_Init(LPUARTx, &LPUART_InitStructure);

    /* Output a message on Hyperterminal using printf function */
    printf("\n\rLPUART Printf Example: retarget the C library printf function to the LPUART mode:%d, src:0x%08x\n\r", _LPUART1_COM_, RCC_GetLpuartClkSrc(LPUARTx_CLK));

    while (1)
    {
    }
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
#if 0            
            RCC_ConfigLSXSEL(RCC_RTCCLK_SRC_LSE);
            /* Configures the External Low Speed oscillator (LSE) */
            RCC_ConfigLse(RCC_LSE_ENABLE);        //32.768KHz
            while (RCC_GetFlagStatus(RCC_LSCTRL_FLAG_LSERD) == RESET)
            {
            }
#else            
            RCC_ConfigLSXSEL(RCC_RTCCLK_SRC_LSI);
            RCC_EnableLsi(ENABLE);        //32KHz
            while (RCC_GetFlagStatus(RCC_LSCTRL_FLAG_LSIRD) == RESET)
            {
            }
#endif
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

    /* Configure LPAURTx Rx as alternate function push-pull and pull-up */
    GPIO_InitStructure.Pin            = LPUARTx_RxPin;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;    
    GPIO_InitStructure.GPIO_Alternate = LPUARTx_Rx_GPIO_AF;
    GPIO_InitPeripheral(LPUARTx_GPIO, &GPIO_InitStructure);
}

/* retarget the C library printf function to the LPUARTx */
int fputc(int ch, FILE* f)
{
    LPUART_SendData(LPUARTx, (uint8_t)ch);
    while (LPUART_GetFlagStatus(LPUARTx, LPUART_FLAG_TXC) == RESET)
        ;
    LPUART_ClrFlag(LPUARTx, LPUART_FLAG_TXC);      

    return (ch);
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
