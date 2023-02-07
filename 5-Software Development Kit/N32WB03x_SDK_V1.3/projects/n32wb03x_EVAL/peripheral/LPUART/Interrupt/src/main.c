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
#include "log.h"

/** @addtogroup N32WB03X_StdPeriph_Examples
 * @{
 */

/** @addtogroup LPUART_Interrupt
 * @{
 */

#define TxBufferSize1 (countof(TxBuffer1))
#define TxBufferSize2 (countof(TxBuffer2))
#define RxBufferSize1 TxBufferSize2
#define RxBufferSize2 TxBufferSize1

#define countof(a) (sizeof(a) / sizeof(*(a)))

LPUART_InitType LPUART_InitStructure;
USART_InitType USART_InitStructure;
uint8_t TxBuffer1[] = "LPUARTy Interrupt Example: LPUARTy -> USARTz using Interrupt";
uint8_t TxBuffer2[] = "LPUARTy Interrupt Example: USARTz -> LPUARTy using Interrupt";
uint8_t RxBuffer1[RxBufferSize1];
uint8_t RxBuffer2[RxBufferSize2];
__IO uint8_t TxCounter1         = 0x00;
__IO uint8_t TxCounter2         = 0x00;
__IO uint8_t RxCounter1         = 0x00;
__IO uint8_t RxCounter2         = 0x00;
uint8_t NbrOfDataToTransfer1    = TxBufferSize1;
uint8_t NbrOfDataToTransfer2    = TxBufferSize2;
uint8_t NbrOfDataToRead1        = RxBufferSize1;
uint8_t NbrOfDataToRead2        = RxBufferSize2;
__IO TestStatus TransferStatus1 = FAILED;
__IO TestStatus TransferStatus2 = FAILED;

/**
 * @brief  Main program
 */
int main(void)
{
    log_init();
    printf("\n This is lpuart interrupt dmoe\r\n");
    
    /* System Clocks Configuration */
    RCC_Configuration(RCC_LPUARTCLK_SRC_APB1);

    /* NVIC configuration */
    NVIC_Configuration();

    /* Configure the GPIO ports */
    GPIO_Configuration();

    /* LPUARTy and USARTz configuration ------------------------------------------------------*/
    
    USART_DeInit(USARTz);    
    USART_InitStructure.BaudRate            = 115200;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;
    /* Configure USARTz */
    USART_Init(USARTz, &USART_InitStructure); 
    /* Enable the USARTz */
    USART_Enable(USARTz, ENABLE);

    LPUART_DeInit(LPUARTy);    
    LPUART_InitStructure.BaudRate            = 115200;
    LPUART_InitStructure.Parity              = LPUART_PE_NO;
    LPUART_InitStructure.RtsThreshold        = LPUART_RTSTH_FIFOFU;
    LPUART_InitStructure.HardwareFlowControl = LPUART_HFCTRL_NONE;
    LPUART_InitStructure.Mode                = LPUART_MODE_RX | LPUART_MODE_TX;
    /* Configure LPUARTy */
    LPUART_Init(LPUARTy, &LPUART_InitStructure);

    /* Flush LPUARTy RX FIFO */
    LPUART_FlushRxFifo(LPUARTy);

    /* Enable LPUARTy Receive and Transmit interrupts */
    LPUART_ConfigInt(LPUARTy, LPUART_INT_FIFO_NE, ENABLE);
    LPUART_ConfigInt(LPUARTy, LPUART_INT_TXC, ENABLE);

    /* Enable USARTz Receive and Transmit interrupts */
    USART_ConfigInt(USARTz, USART_INT_RXDNE, ENABLE);
    USART_ConfigInt(USARTz, USART_INT_TXDE, ENABLE);    

    LPUART_SendData(LPUARTy, TxBuffer1[TxCounter1]);
    /* Wait until end of transmission from LPUARTy to USARTz */
    while (RxCounter2 < RxBufferSize2)
    {
    }
   
    /* Wait until end of transmission from USARTz to LPUARTy */
    while (RxCounter1 < RxBufferSize1)
    {       
    }    

    /* Check the received data with the send ones */
    TransferStatus1 = Buffercmp(TxBuffer2, RxBuffer1, RxBufferSize1);
    /* TransferStatus1 = PASSED, if the data transmitted from USARTz and
       received by LPUARTy are the same */
    /* TransferStatus1 = FAILED, if the data transmitted from USARTz and
       received by LPUARTy are different */
    TransferStatus2 = Buffercmp(TxBuffer1, RxBuffer2, RxBufferSize2);
    /* TransferStatus2 = PASSED, if the data transmitted from LPUARTy and
       received by USARTz are the same */
    /* TransferStatus2 = FAILED, if the data transmitted from LPUARTy and
       received by USARTz are different */
    printf("test result TransferStatus1:%d, TransferStatus2:%d\r\n", TransferStatus1, TransferStatus2);
    printf("RxBuffer1: %s\r\n", RxBuffer1);
    printf("RxBuffer2: %s\r\n", RxBuffer2);
    
    while (1)
    {
    }
}

/**
 * @brief  Configures the different system clocks.
 * @param  LPUART_CLK_SRC: specifies the LPUARTy clock source.
 */
void RCC_Configuration(uint32_t LPUART_CLK_SRC)
{
    USARTz_APBxClkCmd(USARTz_CLK, ENABLE);
	
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
            /* Specifies the LPUARTy clock source, LSE selected as LPUARTy clock */
            RCC_ConfigLpuartClk(LPUARTy_CLK, RCC_LPUARTCLK_SRC_LSI_LSE);
        }
        break;        
        
        default:
        {
            /* Specifies the LPUARTy clock source, APB1 selected as LPUARTy clock */
            RCC_ConfigLpuartClk(LPUARTy_CLK, RCC_LPUARTCLK_SRC_APB1);
        }
        break;
    }    
    
    /* Enable LPUARTy Clock */
    RCC_EnableLpuartClk(ENABLE);    
}

/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;
    
    /* Enable GPIO clock */
    RCC_EnableAPB2PeriphClk(LPUARTy_GPIO_CLK | USARTz_GPIO_CLK | RCC_APB2_PERIPH_AFIO, ENABLE);

    /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);    
    /* Configure LPUARTy Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = LPUARTy_TxPin;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = LPUARTy_Tx_GPIO_AF;
    GPIO_InitPeripheral(LPUARTy_GPIO, &GPIO_InitStructure);

    /* Configure USARTz Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = USARTz_TxPin;
    GPIO_InitStructure.GPIO_Alternate = USARTz_Tx_GPIO_AF;
    GPIO_InitPeripheral(USARTz_GPIO, &GPIO_InitStructure);    

    /* Configure LPUARTy Rx as alternate function push-pull and pull-up */
    GPIO_InitStructure.Pin            = LPUARTy_RxPin;  
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;  
    GPIO_InitStructure.GPIO_Alternate = LPUARTy_Rx_GPIO_AF;
    GPIO_InitPeripheral(LPUARTy_GPIO, &GPIO_InitStructure);

    /* Configure USARTz Rx as alternate function push-pull and pull-up */
    GPIO_InitStructure.Pin            = USARTz_RxPin;
    GPIO_InitStructure.GPIO_Alternate = USARTz_Rx_GPIO_AF;
    GPIO_InitPeripheral(USARTz_GPIO, &GPIO_InitStructure);    
}

/**
 * @brief  Configures the nested vectored interrupt controller.
 */
void NVIC_Configuration(void)
{
    NVIC_InitType NVIC_InitStructure;

    /* Enable the LPUART Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = LPUARTy_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority           = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable the USARTz Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = USARTz_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority           = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  Compares two buffers.
 * @param  pBuffer1, pBuffer2: buffers to be compared.
 * @param BufferLength buffer's length
 * @return PASSED: pBuffer1 identical to pBuffer2
 *         FAILED: pBuffer1 differs from pBuffer2
 */
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
    while (BufferLength--)
    {
        if (*pBuffer1 != *pBuffer2)
        {
            return FAILED;
        }

        pBuffer1++;
        pBuffer2++;
    }

    return PASSED;
}



/**
 * @}
 */

/**
 * @}
 */
