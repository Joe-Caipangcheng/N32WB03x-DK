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
 * @file dfu_usart.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** @addtogroup 
 * @{
 */

 /* Includes ------------------------------------------------------------------*/
#include "dfu_usart.h"
#include "n32wb03x.h"
#include <stdio.h>


/**
 * @brief  config rcc of usart1
 * @param  
 * @return 
 * @note   
 */
static void rcc_configure_usart1(void)
{
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_USART1, ENABLE);
}

/**
 * @brief  config gpio of usart1
 * @param  
 * @return 
 * @note   
 */
static void gpio_configure_usart1(void)
{
    GPIO_InitType GPIO_InitStructure;
    
    GPIO_InitStruct(&GPIO_InitStructure);
    
    GPIO_InitStructure.Pin              =    GPIO_PIN_6;
    GPIO_InitStructure.GPIO_Mode        =    GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate   =    GPIO_AF4_USART1;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
    
    
    GPIO_InitStructure.Pin              =    GPIO_PIN_7;
    GPIO_InitStructure.GPIO_Alternate   =    GPIO_AF4_USART1;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);    
    
    
}

/**
 * @brief  config usart1
 * @param  
 * @return 
 * @note   
 */
static void config_usart1(void)
{
    USART_InitType USART_InitStructure;
    
    USART_InitStructure.BaudRate                   =      115200;
    USART_InitStructure.WordLength                 =      USART_WL_8B;
    USART_InitStructure.StopBits                   =      USART_STPB_1;
    USART_InitStructure.Parity                     =      USART_PE_NO;
    USART_InitStructure.HardwareFlowControl        =      USART_HFCTRL_NONE;
    USART_InitStructure.Mode                       =      USART_MODE_RX | USART_MODE_TX;
    
    USART_Init(USART1, &USART_InitStructure);

}

/**
 * @brief  config usart1 for dfu 
 * @param  
 * @return 
 * @note   
 */
void dfu_usart1_config(void)
{
    rcc_configure_usart1();
    gpio_configure_usart1();
    config_usart1();
}

/**
 * @brief  enable usart1
 * @param  
 * @return 
 * @note   
 */
void dfu_usart1_enable(void)
{
    USART_Enable(USART1, ENABLE);
}


/**
 * @brief  disable usart1
 * @param  
 * @return 
 * @note   
 */
void dfu_usart1_disable(void)
{
    USART_Enable(USART1, DISABLE);
}

/**
 * @brief  send data via usart1
 * @param  
 * @return 
 * @note   
 */
void dfu_usart1_send(uint8_t *p_data, uint32_t len)
{
    for(uint32_t i=0;i<len;i++)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXDE) != SET);
        USART_SendData(USART1, p_data[i]);
    }
}


/**
 * @brief  receive data from usart1
 * @param  
 * @return 
 * @note   
 */
uint8_t dfu_usart1_receive(uint8_t *p_data)
{
    if(USART_GetFlagStatus(USART1, USART_FLAG_RXDNE) != RESET)
    {
        *p_data = USART_ReceiveData(USART1);
        return 1;
    }
    return 0;
}


/**
 * @brief  config nvic of usart1
 * @param  
 * @return 
 * @note   
 */
static void nvic_configure_usart1(void)
{
    NVIC_InitType NVIC_InitStructure;
    
    NVIC_InitStructure.NVIC_IRQChannel              =            USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority      =            0;
    NVIC_InitStructure.NVIC_IRQChannelCmd           =            ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

/**
 * @brief  enable interrupt of usart1
 * @param  
 * @return 
 * @note   
 */
static void config_interrupt_usart1(void)
{
    USART_ConfigInt(USART1, USART_INT_RXDNE, ENABLE);
}

/**
 * @brief  config usart1 and enable interrupt 
 * @param  
 * @return 
 * @note   
 */
void dfu_usart1_interrupt_config(void)
{
    rcc_configure_usart1();
    gpio_configure_usart1();
    config_usart1();
    nvic_configure_usart1();
    config_interrupt_usart1();
}







