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

#include "global_func.h"
#include "app_usart.h"
#include "ns_log.h"
#include "ns_delay.h"
#include "app_ble.h"
/** @addtogroup 
 * @{
 */
/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    STATE_UNINITIALIZED,
    STATE_IDLE,
    STATE_TRANSMITTER_TEST,
    STATE_CARRIER_TEST,
    STATE_RECEIVER_TEST
} state_t;
/* Private define ------------------------------------------------------------*/

//#define LE_PACKET_REPORTING_EVENT       0x8000                             
#define RF_TEST_STATUS_EVENT_SUCCESS    0x0000                             
#define RF_TEST_STATUS_EVENT_ERROR      0x0001                             

#define CMD_RSP_SUCCESS                     0x00                               
#define CMD_RSP_ERROR_ILLEGAL_CHANNEL       0x01                               
#define CMD_RSP_ERROR_INVALID_STATE         0x02                               
#define CMD_RSP_ERROR_ILLEGAL_LENGTH        0x03                               
#define CMD_RSP_ERROR_ILLEGAL_CONFIGURATION 0x04                               
#define CMD_RSP_ERROR_UNINITIALIZED         0x05    

#define RF_TEST_SETUP                   0                                  
#define RF_RECEIVER_TEST                1                                  
#define RF_TRANSMITTER_TEST             2                                  
#define RF_TEST_END                     3

#define RF_TEST_SETUP_RESET             0                                  
#define RF_TEST_SETUP_SET_UPPER         1                                  
#define RF_TEST_SETUP_SET_PHY           2                                  
#define RF_TEST_SETUP_SELECT_MODULATION 3                                  
#define RF_TEST_SETUP_READ_SUPPORTED    4                                  
#define RF_TEST_SETUP_READ_MAX          5                                  
                              
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
    USART_InitStructure.BaudRate            = 115200;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;

    /* Configure USARTx */
    USART_Init(USARTx, &USART_InitStructure);

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
        rx_old_pos = 0;
        usart_tx_fifo_in = usart_tx_fifo_out = 0; //clean fifo
        usart_rx_fifo_in = usart_rx_fifo_out = 0; //clean fifo
        app_usart_configuration();
//        m_state         = STATE_IDLE;
    }
    else{

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
 * @brief  usart send data in blocking mode
 */
void usart_tx_data_blocking(uint8_t *p_data, uint16_t len)
{
    while(len--)
    {
         USART_SendData(USARTx, *p_data);
         while(USART_GetFlagStatus(USARTx,USART_FLAG_TXC) == RESET);
         p_data++;
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

        DMA_ClearFlag(DMA_FLAG_TC1, DMA);
    }
    //RX 
    if(DMA_GetFlagStatus(DMA_FLAG_TC2, DMA))
    {
        //RX Transfer complete interrupt
        DMA_ClearFlag(DMA_FLAG_TC2, DMA);
//        usart_rx_check_in_irq();
        
    }
    if(DMA_GetFlagStatus(DMA_FLAG_HT2, DMA))
    {
        //RX Half transfer interrupt 
        DMA_ClearFlag(DMA_FLAG_HT2, DMA);
//        usart_rx_check_in_irq();
    }


}



uint8_t  g_send_interval_recv_flag =0;
#include "string.h"
#include "h4tl.h"
#define BYTES_TO_UINT16(n, p)                         \
{                                                     \
    n = ((uint16_t)(p)[0] + ((uint16_t)(p)[1] << 8)); \
}

void flash_utils_flow_on(void) 
{

}    

void flash_utils_flow_off(void)
{

}   

struct utils_txrxchannel
{
    /// call back function pointer
    void (*callback)(void*, uint8_t);
    /// Dummy data pointer returned to callback when operation is over.
    void* dummy;
};

/// UART environment structure
struct utils_env_tag
{
    /// tx channel
    struct utils_txrxchannel tx;
    /// rx channel
    struct utils_txrxchannel rx;
    /// error detect
    uint8_t errordetect;
    /// external wakeup
    bool ext_wakeup;
};
struct utils_env_tag flash_utils_env;

uint16_t  flash_g_recv_len = 0;
extern uint8_t USART_rx_dma_buf[USART_RX_DMA_SIZE] ;
void flash_utils_hci_cmd_reset(void)
{
    flash_g_recv_len = 0;
    
    memset(USART_rx_dma_buf, 0x00, 8);
    extern void DMA_Configuration(void);
    DMA_Configuration();
    DMA_EnableChannel(USARTx_Rx_DMA_Channel, ENABLE);
}



uint16_t flash_utils_get_tx_len(void)
{
    return USART_RX_DMA_SIZE - DMA_GetCurrDataCounter(USARTx_Rx_DMA_Channel); 
}

void flash_utils_read (uint8_t *bufptr, uint32_t size, rwip_eif_callback callback, void* dummy)
{
    NS_LOG_INFO("%s\r\n",__func__);
    flash_utils_env.rx.callback = callback;
    flash_utils_env.rx.dummy = dummy;
    uint32_t i;
    uint8_t rx_state = 0;
    //uint16_t payload_len = 0;
    //uint8_t* dummytrans = utils_env.rx.dummy;
    struct h4tl_env_tag *dummy_env = (struct h4tl_env_tag *)dummy;
    rx_state =  dummy_env->rx_state ; // dummytrans[12]; //RX_STATE is the 12th byte in the h4tl_env_tag structure
    if (rx_state == H4TL_STATE_RX_START) //The first byte to receive
    {
        __NOP();
    } 
    else if (rx_state == H4TL_STATE_RX_HDR) //Receive header
    {
        if (size)
        {
            //Wait until enough length header been received
            i = 0;
            while (flash_utils_get_tx_len() < size + 1)
            {
                delay_n_10us(10);
                if (i++ > 10000)
                {
                    break;
                }
            }
        }
    }
    else if ((rx_state == H4TL_STATE_RX_PAYL)) //Receive payload
    {
        if (size)
        {
            //Wait until enough length payload been received
            i = 0;
            while (flash_utils_get_tx_len() < flash_g_recv_len + size)
            {
                delay_n_ms(1);
                if (i++ > 5000) //5s timeout
                {
                    break ;
                }
                   
            }
        }
    }
    /*Clear buffer and reset state when out of sync*/
    else if (rx_state == H4TL_STATE_RX_OUT_OF_SYNC)
    {
        while (flash_utils_get_tx_len() > 0)
        {
            flash_utils_hci_cmd_reset();
            delay_n_10us(50);
        }
        //dummytrans[12] = H4TL_STATE_RX_START;
        dummy_env->rx_state = H4TL_STATE_RX_START;    
        flash_utils_env.rx.callback(flash_utils_env.rx.dummy, 3);
        return;
    }
    /*Buffer copy*/
    for (i = 0; i < size; i++)
    {
        if (i + flash_g_recv_len < utils_max_buf_len)
        {
            bufptr[i] = USART_rx_dma_buf[i + flash_g_recv_len];
        }
    }
    flash_g_recv_len += size;
    
    int recv_flag = 0;
    /*Parse command header for the length of payload*/
    if (rx_state == H4TL_STATE_RX_HDR)
    {
        uint16_t payload_len=0;
        uint8_t rx_type = USART_rx_dma_buf[0];
        if(rx_type == HCI_CMD_MSG_TYPE)
        {
            payload_len=USART_rx_dma_buf[3];
        }
        else if(rx_type==HCI_ACL_MSG_TYPE) 
        {
            BYTES_TO_UINT16(payload_len, &USART_rx_dma_buf[3]);
        }
        else if(rx_type==AHI_KE_MSG_TYPE)
        {
            BYTES_TO_UINT16(payload_len, &USART_rx_dma_buf[7]);
        }
        
        if(payload_len == 0)
        {    
            recv_flag = 1;
        }
    }
    
    //Accumulate the length
    if ((rx_state == H4TL_STATE_RX_PAYL)) //Done
    {
        recv_flag = 1;
    }   
 
    // Call handler
    flash_utils_env.rx.callback(flash_utils_env.rx.dummy, RWIP_EIF_STATUS_OK);
    if(recv_flag)
    {
        flash_utils_hci_cmd_reset();
    }

}


void flash_utils_write(uint8_t* bufptr, uint32_t size, void (*callback)(void*, uint8_t), void* dummy)
{
    //uint8_t *temp_bufptr;
    flash_utils_env.tx.callback = callback;
    flash_utils_env.tx.dummy = dummy;
    
    /* Check if there is more than 1 hci command in the buffer*/
    //if more data in buffer then to handle
    rwip_prevent_sleep_set(RW_TL_RX_ONGOING);

    usart_tx_data_blocking(bufptr, size);
  
    //printf("g_send_interval_recv_flag\r\n");
    flash_utils_hci_cmd_reset();    
      
    /*tx done */
    flash_utils_env.tx.callback(flash_utils_env.tx.dummy, RWIP_EIF_STATUS_OK);
}
 



//Creation of uart external interface api
const struct rwip_eif_api flash_utils_api =
{
    flash_utils_read,
    flash_utils_write,
    flash_utils_flow_on,
    flash_utils_flow_off,
};
   
bool flash_utils_hci_cmd_check(void)
{
    if ((flash_utils_get_tx_len() >= 4) && ((USART_rx_dma_buf[0] == HCI_CMD_MSG_TYPE) || (USART_rx_dma_buf[0] == HCI_ACL_MSG_TYPE) || (USART_rx_dma_buf[0] == AHI_KE_MSG_TYPE) ))
    {
        rwip_prevent_sleep_set(RW_TL_RX_ONGOING);
        return true;
    }
    else
    {
        /*Wrong packet type*/
        if ((USART_rx_dma_buf[0] != 0) && (USART_rx_dma_buf[0] != HCI_CMD_MSG_TYPE) && (USART_rx_dma_buf[0] != HCI_ACL_MSG_TYPE)  &&  (USART_rx_dma_buf[0] != AHI_KE_MSG_TYPE)  ) 
        {
            while (flash_utils_get_tx_len() > 0)
            {
                flash_utils_hci_cmd_reset();
                delay_n_10us(50);
            }
        }
        rwip_prevent_sleep_clear(RW_TL_RX_ONGOING);
        return false;
    }
}


const struct rwip_eif_api* rwip_eif_get_flash(uint8_t type)
{
    
    return &flash_utils_api;
}


/**
 * @}
 */

