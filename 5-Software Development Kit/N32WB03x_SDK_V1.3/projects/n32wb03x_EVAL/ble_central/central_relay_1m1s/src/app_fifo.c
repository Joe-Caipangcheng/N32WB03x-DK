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
 * @file app_fifo.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "app_fifo.h"
#include "app_rdtss.h"
#include "rwprf_config.h"
#include "string.h"
#include "ke_timer.h"
#include "rdtss_task.h"

/** @addtogroup 
 * @{
 */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

uint8_t  master_rx_fifo_buf[MASTER_RX_FIFO_SIZE] = {0};
uint32_t master_rx_fifo_in = 0;
uint32_t master_rx_fifo_out = 0;
bool sending_to_master;
uint16_t num_send_to_peer_master = 20;      //MTU-3

/**
 * @brief  pop own master RX data, send data to peer master
 */
void pop_matser_rx_to_ble(void)
{
    uint32_t in_temp;
    uint16_t ble_send_len;
    
    in_temp = master_rx_fifo_in;
    if(master_rx_fifo_out < in_temp)
    {
        ble_send_len = in_temp-master_rx_fifo_out;
    }
    else if(master_rx_fifo_out > in_temp){
        ble_send_len = MASTER_RX_FIFO_SIZE-master_rx_fifo_out;
    }
    else if(master_rx_fifo_out == in_temp){
        // fifo empty, stop send loop
        sending_to_master = false;
        return;
    }
    if(ble_send_len > num_send_to_peer_master)
    {
        ble_send_len = num_send_to_peer_master;
    }
    sending_to_master = true;    
    #if (BLE_RDTSS_SERVER)
    rdtss_send_notify(  &master_rx_fifo_buf[master_rx_fifo_out], ble_send_len);
    #endif
    master_rx_fifo_out = (master_rx_fifo_out+ble_send_len)%MASTER_RX_FIFO_SIZE;

    return;
}

/**
 * @brief  push master RX data to fifo and active ble send first package if not active yet
 */
uint8_t push_master_rx_data_to_fifo(const uint8_t *p_data, uint16_t len)
{
    uint32_t in_len;
    //store data in fifo
    while(len)
    {
        if(master_rx_fifo_in >= master_rx_fifo_out )
        {
            in_len = MASTER_RX_FIFO_SIZE-master_rx_fifo_in;
            if(in_len > len)
            {
                in_len = len;
            }
            memcpy(&master_rx_fifo_buf[master_rx_fifo_in],p_data,in_len);
            len = len-in_len;
            p_data += in_len;
            master_rx_fifo_in = (master_rx_fifo_in + in_len)%MASTER_RX_FIFO_SIZE;
            
        }
        else if(master_rx_fifo_in < master_rx_fifo_out )
        {
            in_len = master_rx_fifo_out-master_rx_fifo_in-1;
            if(in_len > len)
            {
                in_len = len;
            }
            memcpy(&master_rx_fifo_buf[master_rx_fifo_in],p_data,in_len);
            len = len-in_len;
            master_rx_fifo_in = (master_rx_fifo_in + in_len)%MASTER_RX_FIFO_SIZE;
            
            //fifo full,drop the rest data 
            if(len)
            {
                NS_LOG_WARNING("F:%d,%d,%d\r\n",len,master_rx_fifo_in,master_rx_fifo_out);
            }
            break;
        }
    }
    
    if(master_rx_fifo_in != master_rx_fifo_out)       //data need to send
    {
        if(!sending_to_master)
        {
            ke_timer_set(RDTSS_VAL_NTF_CFM, TASK_APP, 10);
        }
    }

    return len;
}



/**
 * @}
 */
