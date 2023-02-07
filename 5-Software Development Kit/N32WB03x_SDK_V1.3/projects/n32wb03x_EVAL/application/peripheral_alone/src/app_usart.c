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
 * @file app_usart.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "app_usart.h"
#include "main.h"
#include <stdio.h>


/** @addtogroup 
 * @{
 */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t USART_fifo_buf[256] = {0};
__IO uint8_t USART_fifo_in = 0;
__IO uint8_t USART_fifo_out = 0;
/* Private function prototypes -----------------------------------------------*/



/* Private functions ---------------------------------------------------------*/



/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration_USART(void)
{
    /* Enable GPIO clock */
    GPIO_APBxClkCmd(USARTx_GPIO_CLK, ENABLE);
    /* Enable USARTx Clock */
    USART_APBxClkCmd(USARTx_CLK, ENABLE);
}

/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration_USART(void)
{
    GPIO_InitType GPIO_InitStructure;

    /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);    

    /* Configure USARTx Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = USARTx_TxPin;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = USARTx_Tx_GPIO_AF;
    GPIO_InitPeripheral(USARTx_GPIO, &GPIO_InitStructure);   

    /* Configure USARTx Rx as alternate function push-pull */
    GPIO_InitStructure.Pin            = USARTx_RxPin;
    GPIO_InitStructure.GPIO_Alternate = USARTx_Rx_GPIO_AF;
    GPIO_InitPeripheral(USARTx_GPIO, &GPIO_InitStructure); 
}

/**
 * @brief  Deinitializes the USART.
 */
void USART_Deinitializes(void)
{
    GPIO_InitType GPIO_InitStructure;
    USART_Enable(USARTx, DISABLE);

    /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);    

    /* Configure USARTx Tx Rx as ANALOG to save power */
    GPIO_InitStructure.Pin            = USARTx_TxPin|USARTx_RxPin;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_ANALOG;
    GPIO_InitPeripheral(USARTx_GPIO, &GPIO_InitStructure);   


}

/**
 * @brief  Configures the USART as 115200 8n1.
 */
void USART_Configuration(void)
{

    USART_InitType USART_InitStructure;
         
    /* Configure and enable RCC */
    RCC_Configuration_USART();
    /* Configure GPIO for USART */
    GPIO_Configuration_USART();

        /* USARTy and USARTz configuration ------------------------------------------------------*/
    USART_InitStructure.BaudRate            = 115200;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;

    /* Configure USARTx */
    USART_Init(USARTx, &USART_InitStructure);
    /* Enable the USARTx */
    USART_Enable(USARTx, ENABLE);

}

/**
 * @brief  USART
 */
void USART_EchoTask(void)
{
    /* USART receive */
    if(USART_GetFlagStatus(USARTx, USART_FLAG_RXDNE) != RESET)
    {
        /*buffer not full yet*/
        if((USART_fifo_in+1) != USART_fifo_out)
        {
            /* Store the received byte in USART_fifo_buf */
            USART_fifo_buf[USART_fifo_in++] = USART_ReceiveData(USARTx);            
        }
    }
    
        /* USART send */
    if(USART_GetFlagStatus(USARTx, USART_FLAG_TXDE) != RESET)
    {
        /*buffer not empty yet*/
        if(USART_fifo_out != USART_fifo_in)
        {
            /*Send one byte to USART from buf */
            USART_SendData(USARTx, USART_fifo_buf[USART_fifo_out++]);
        }
    }

}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE* f)
{
    USART_SendData(USARTx, (uint8_t)ch);
    while (USART_GetFlagStatus(USARTx, USART_FLAG_TXDE) == RESET);

    return (ch);
}



/**
 * @}
 */

