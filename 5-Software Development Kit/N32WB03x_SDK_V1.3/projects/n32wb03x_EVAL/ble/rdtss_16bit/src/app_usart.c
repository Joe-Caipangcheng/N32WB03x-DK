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
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include <string.h>
#include <stdio.h>
#include "global_func.h"
#include "app_usart.h"
#include "ns_sleep.h"
#include "ns_log.h"
#include "ns_timer.h"
#include "ke_timer.h"
#include "app_ble.h"   

#if (CFG_PRF_RDTSS)
#include "rdtss_task.h" 
#include "app_rdtss.h"
#endif
#if (CFG_PRF_RDTSS_16BIT)
#include "rdtss_16bit_task.h" 
#include "app_rdtss_16bit.h"
#endif
/** @addtogroup 
 * @{
 */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t USART_rx_dma_buf[USART_RX_DMA_SIZE] = {0};
uint16_t rx_old_pos = 0;

uint8_t  usart_rx_fifo_buf[USART_RX_FIFO_SIZE] = {0};
uint32_t usart_rx_fifo_in = 0;
uint32_t usart_rx_fifo_out = 0;


uint8_t  usart_tx_fifo_buf[USART_TX_FIFO_SIZE] = {0};
uint32_t usart_tx_fifo_in = 0;
uint32_t usart_tx_fifo_out = 0;

uint8_t usart_sending = false;
uint8_t ble_sending   = false;
uint16_t ble_att_mtu = (ATT_DEFAULT_MTU-3);

/* Private function prototypes -----------------------------------------------*/
void app_usart_tx_process(void);

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
        /* DMA clock enable */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_DMA, ENABLE);
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
 * @brief  Configures the DMA.
 */
void DMA_Configuration(void)
{
    DMA_InitType DMA_InitStructure;

    /* USARTx_Tx_DMA_Channel (triggered by USARTx Tx event) Config */
    DMA_DeInit(USARTx_Tx_DMA_Channel);
    DMA_RequestRemap(USARTx_Tx_DMA_REMAP, DMA, USARTx_Tx_DMA_Channel, ENABLE);    
    DMA_ConfigInt(USARTx_Tx_DMA_Channel,DMA_INT_TXC ,ENABLE);

    /* USARTx RX DMA1 Channel (triggered by USARTx Rx event) Config */
    DMA_DeInit(USARTx_Rx_DMA_Channel);
    DMA_RequestRemap(USARTx_Rx_DMA_REMAP, DMA, USARTx_Rx_DMA_Channel, ENABLE);        
    DMA_ConfigInt(USARTx_Rx_DMA_Channel,DMA_INT_TXC|DMA_INT_HTX ,ENABLE);
        
    DMA_InitStructure.PeriphAddr     = USARTx_DAT_Base;
    DMA_InitStructure.MemAddr        = (uint32_t)USART_rx_dma_buf;
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_SRC;
    DMA_InitStructure.BufSize        = USART_RX_DMA_SIZE;
    DMA_InitStructure.CircularMode   = DMA_MODE_CIRCULAR;
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_DISABLE;
    DMA_InitStructure.DMA_MemoryInc  = DMA_MEM_INC_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
    DMA_InitStructure.MemDataSize    = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.Priority       = DMA_PRIORITY_VERY_HIGH;
    DMA_InitStructure.Mem2Mem        = DMA_M2M_DISABLE;
    DMA_Init(USARTx_Rx_DMA_Channel, &DMA_InitStructure);

}


/**
 * @brief  Configures the nested vectored interrupt controller.
 */
void NVIC_Configuration(void)
{
    NVIC_InitType NVIC_InitStructure;

    NVIC_DisableIRQ(DMA_Channel1_2_3_4_IRQn);
    /* Enable the DMA Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = DMA_Channel1_2_3_4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority           = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_DisableIRQ(USARTx_IRQn);
    /* Enable the DMA Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = USARTx_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority           = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

/**
 * @brief  Configures the USART as 115200 8n1.
 */
void app_usart_configuration(void)
{

    USART_InitType USART_InitStructure;
         
    /* Configure and enable RCC */
    RCC_Configuration_USART();
    /* Configure GPIO for USART */
    GPIO_Configuration_USART();

    /* Configure the DMA */
    DMA_Configuration();

    NVIC_Configuration();
    
    /* USARTx and USARTz configuration ------------------------------------------------------*/
    USART_InitStructure.BaudRate            = 115200; //921600
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;

    /* Configure USARTx */
    USART_Init(USARTx, &USART_InitStructure);
        
    /* Enable USARTz Receive and Transmit interrupts */
    USART_ConfigInt(USARTx, USART_INT_IDLEF, ENABLE);

    /* Enable USARTx DMA Rx and TX request */
    USART_EnableDMA(USARTx, USART_DMAREQ_RX | USART_DMAREQ_TX, ENABLE);
    
    DMA_EnableChannel(USARTx_Rx_DMA_Channel, ENABLE);
    
    /* Enable the USARTx */
    USART_Enable(USARTx, ENABLE);
    
}

/**
 * @brief  enable or disable the usart dma
 */
void app_usart_dma_enable(FunctionalState Cmd)
{
    if(Cmd == ENABLE)
    {
        ns_sleep_lock_acquire();
        rx_old_pos = 0;
        usart_tx_fifo_in = usart_tx_fifo_out = 0; //clean fifo
        usart_rx_fifo_in = usart_rx_fifo_out = 0; //clean fifo
        app_usart_configuration();

    }
    else{
        ns_sleep_lock_release();
        USART_Enable(USARTx, DISABLE);
        DMA_EnableChannel(USARTx_Rx_DMA_Channel, DISABLE);
        DMA_EnableChannel(USARTx_Tx_DMA_Channel, DISABLE);
        /* Deinit IO of USART to save power */
        GPIO_InitType GPIO_InitStructure;
        /* Initialize GPIO_InitStructure */
        GPIO_InitStruct(&GPIO_InitStructure);    
        /* Configure USARTx Tx as alternate function push-pull */
        GPIO_InitStructure.Pin            = USARTx_TxPin|USARTx_RxPin;
        GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_ANALOG;
        GPIO_InitStructure.GPIO_Alternate = GPIO_AF0;
        GPIO_InitPeripheral(USARTx_GPIO, &GPIO_InitStructure);  
    }
}

/**
 * @brief  forward usart rx data too ble notify
 */
void usart_forward_to_ble_loop(void)
{
    uint32_t in_temp;
    uint16_t ble_send_len;
    
    in_temp = usart_rx_fifo_in;
    if(usart_rx_fifo_out < in_temp)
    {
        ble_send_len = in_temp-usart_rx_fifo_out;
    }
    else if(usart_rx_fifo_out > in_temp){
        ble_send_len = USART_RX_FIFO_SIZE-usart_rx_fifo_out;
    }
    else if(usart_rx_fifo_out == in_temp){
        // fifo empty, stop send loop
        ble_sending = false;
        return;
    }
    if(ble_send_len > ble_att_mtu)
    {
        ble_send_len = ble_att_mtu;
    }
    ble_sending = true;    
    #if (BLE_RDTSS_SERVER)
    rdtss_send_notify(  &usart_rx_fifo_buf[usart_rx_fifo_out], ble_send_len);
    #endif
    
    #if (BLE_RDTSS_16BIT_SERVER)
    rdtss_16bit_send_notify(  &usart_rx_fifo_buf[usart_rx_fifo_out], ble_send_len);
    #endif
    usart_rx_fifo_out = (usart_rx_fifo_out+ble_send_len)%USART_RX_FIFO_SIZE;

    return;
}


/**
 * @brief  usart rx data enter fifo and active ble send first package if not active yet
 */
uint8_t app_usart_rx_data_fifo_enter(const uint8_t *p_data, uint16_t len)
{
    uint32_t in_len;
    //store data in fifo
    while(len)
    {
        if(usart_rx_fifo_in >= usart_rx_fifo_out )
        {
            in_len = USART_RX_FIFO_SIZE-usart_rx_fifo_in;
            if(in_len > len)
            {
                in_len = len;
            }
            memcpy(&usart_rx_fifo_buf[usart_rx_fifo_in],p_data,in_len);
            len = len-in_len;
            p_data += in_len;
            usart_rx_fifo_in = (usart_rx_fifo_in + in_len)%USART_RX_FIFO_SIZE;
            
        }
        else if(usart_rx_fifo_in < usart_rx_fifo_out )
        {
            in_len = usart_rx_fifo_out-usart_rx_fifo_in-1;
            if(in_len > len)
            {
                in_len = len;
            }
            memcpy(&usart_rx_fifo_buf[usart_rx_fifo_in],p_data,in_len);
            len = len-in_len;
            usart_rx_fifo_in = (usart_rx_fifo_in + in_len)%USART_RX_FIFO_SIZE;
            
            //fifo full,drop the rest data 
            if(len)
            {
                NS_LOG_WARNING("F:%d,%d,%d\r\n",len,usart_rx_fifo_in,usart_rx_fifo_out);
            }
            break;
        }
    }
    if(!ble_sending)
    {
        // active send ble event after usart streaming cut
        ke_timer_set(RDTSS_16BIT_VAL_NTF_CFM, TASK_APP, 10);
    }

    return len;
}

/**
 * @brief  usart send data via dma
 */
uint8_t usart_tx_dma_send(uint8_t *p_data, uint16_t len)
{
    DMA_InitType DMA_InitStructure;

    if(usart_sending)
    {
        return false;
    }
    /* USARTx_Tx_DMA_Channel (triggered by USARTx Tx event) Config */
    DMA_InitStructure.PeriphAddr     = USARTx_DAT_Base;
    DMA_InitStructure.MemAddr        = (uint32_t)p_data;
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_DST;
    DMA_InitStructure.BufSize        = len;
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_DISABLE;
    DMA_InitStructure.DMA_MemoryInc  = DMA_MEM_INC_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
    DMA_InitStructure.MemDataSize    = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.CircularMode   = DMA_MODE_NORMAL; 
    DMA_InitStructure.Priority       = DMA_PRIORITY_VERY_HIGH;
    DMA_InitStructure.Mem2Mem        = DMA_M2M_DISABLE;
    DMA_Init(USARTx_Tx_DMA_Channel, &DMA_InitStructure);

    usart_sending = true;
    DMA_EnableChannel(USARTx_Tx_DMA_Channel, ENABLE);
     
    return true;
}

/**
 * @brief  usart send data in blocking mode
 */
void usart_tx_data_blocking(uint8_t *p_data, uint16_t len)
{
    while(len--)
    {
         USART_SendData(USARTx, *p_data);
         p_data++;
    }
}

/**
 * @brief  usart tx fifo enter and active dma send out
 */
uint8_t app_usart_tx_fifo_enter(const uint8_t *p_data, uint16_t len)
{
    uint32_t in_len, out_temp;
    //store data in fifo
    NS_LOG_DEBUG("%d,%d,%d\r\n",len,usart_tx_fifo_in,usart_tx_fifo_out);
    out_temp = usart_tx_fifo_out;
    while(len)
    {
        if(usart_tx_fifo_in >= out_temp )
        {
            in_len = USART_TX_FIFO_SIZE-usart_tx_fifo_in;
            if(in_len > len)
            {
                in_len = len;
            }
            memcpy(&usart_tx_fifo_buf[usart_tx_fifo_in],p_data,in_len);
            len = len-in_len;
            p_data +=  in_len;
            usart_tx_fifo_in = (usart_tx_fifo_in + in_len)%USART_TX_FIFO_SIZE;
            
        }
        else if(usart_tx_fifo_in < out_temp )
        {
            in_len = out_temp-usart_tx_fifo_in-1;
            if(in_len > len)
            {
                in_len = len;
            }
            memcpy(&usart_tx_fifo_buf[usart_tx_fifo_in],p_data,in_len);
            len = len-in_len;
            usart_tx_fifo_in = (usart_tx_fifo_in + in_len)%USART_TX_FIFO_SIZE;
            
            //fifo full,drop the rest data 
            if(len)
            {
                NS_LOG_WARNING("F:%d,%d,%d\r\n",len,usart_tx_fifo_in,out_temp);
            }
            
            break;
        }
    }
    
    //  ble treaming cut timer, active usart send after it.
    ke_timer_set(APP_CUSTS_TEST_EVT, TASK_APP, 50);

    return len;
}

/**
 * @brief  usart tx data from fifo
 */
void app_usart_tx_process(void)
{
    uint32_t in_temp,len;
    uint8_t *p_data;
    
    in_temp = usart_tx_fifo_in;
    if(usart_tx_fifo_out < in_temp)
    {
        len = in_temp-usart_tx_fifo_out;
        p_data = &usart_tx_fifo_buf[usart_tx_fifo_out];
        if(usart_tx_dma_send(p_data,len) == true)
        {
            usart_tx_fifo_out = in_temp;
        }

    }
    else if(usart_tx_fifo_out > in_temp)
    {
        len = USART_TX_FIFO_SIZE-usart_tx_fifo_out;
        p_data = &usart_tx_fifo_buf[usart_tx_fifo_out];
        if(usart_tx_dma_send(p_data,len) == true)
        {
            usart_tx_fifo_out = 0;
        }
       
    }
}


/**
 * @brief  check the dma buffer which has been received 
 */
void usart_rx_check_in_irq(void)
{
    uint16_t rx_pos;

    rx_pos = USART_RX_DMA_SIZE - DMA_GetCurrDataCounter(USARTx_Rx_DMA_Channel);
    if(rx_pos <  rx_old_pos)
    {
        app_usart_rx_data_fifo_enter(&USART_rx_dma_buf[rx_old_pos],(USART_RX_DMA_SIZE - rx_old_pos));
        
        if(rx_pos > 0)
        {
            app_usart_rx_data_fifo_enter(&USART_rx_dma_buf[0],rx_pos);
        }
        
        rx_old_pos = rx_pos;
    }
    else if(rx_pos >  rx_old_pos)
    {
        app_usart_rx_data_fifo_enter(&USART_rx_dma_buf[rx_old_pos],(rx_pos - rx_old_pos));
        rx_old_pos = rx_pos;
    }
    else if(rx_pos != rx_old_pos)
    {
        //error
        rx_old_pos = rx_pos;
    }

}


/**
 * @brief  dma irq handler
 */
void DMA_Channel1_2_3_4_IRQHandler(void)
{
    //TX
    if(DMA_GetFlagStatus(DMA_FLAG_TC1, DMA))
    {
        //TX Transfer complete interrupt
        usart_sending = false;
        ke_msg_send_basic(APP_CUSTS_TEST_EVT, TASK_APP, TASK_APP);

        DMA_ClearFlag(DMA_FLAG_TC1, DMA);
    }
    //RX 
    if(DMA_GetFlagStatus(DMA_FLAG_TC2, DMA))
    {
        //RX Transfer complete interrupt
        DMA_ClearFlag(DMA_FLAG_TC2, DMA);
        usart_rx_check_in_irq();
        
    }
    if(DMA_GetFlagStatus(DMA_FLAG_HT2, DMA))
    {
        //RX Half transfer interrupt 
        DMA_ClearFlag(DMA_FLAG_HT2, DMA);
        usart_rx_check_in_irq();
    }

}

/**
 * @brief  usart irq handler
 */
void USARTx_IRQHandler(void)
{
    uint8_t temp;
    //usart idlle interrupt
    if(USART_GetFlagStatus(USARTx,USART_FLAG_IDLEF))
    {
        //read sts and data will clear rx idle interrupt
        temp = USARTx->DAT;
        usart_rx_check_in_irq();
        (void)temp;
    }

}



/**
 * @}
 */

