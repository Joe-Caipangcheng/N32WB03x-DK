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
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** @addtogroup 
 * @{
 */

 /* Includes ------------------------------------------------------------------*/
#include "dfu_led.h"
#include "n32wb03x.h"
#include "ns_scheduler.h"
#include "ns_dfu_boot.h"
#include "dfu_usart.h"
#include "dfu_delay.h"
#include "ns_error.h"




static uint32_t serial_send_data(uint8_t *p_data, uint32_t length);




/**
 * @brief  main function
 * @param   
 * @return 
 * @note   Note
 */
int main(void)
{
    PWR->VTOR_REG = CURRENT_APP_START_ADDRESS | 0x80000000;
    
    dfu_leds_config();
    if(CURRENT_APP_START_ADDRESS == NS_APP1_START_ADDRESS){
        dfu_led_on(LED1_GPIO_PORT, LED_GPIO1_PIN);
    }else if(CURRENT_APP_START_ADDRESS == NS_APP2_START_ADDRESS){
        dfu_led_on(LED2_GPIO_PORT, LED_GPIO2_PIN);
    }    
    
    NS_SCHED_INIT(256, 16);    

    Qflash_Init();
    dfu_usart1_interrupt_config();
    dfu_usart1_enable();    
    while(1)
    {
        app_sched_execute();
        __WFE();
        __SEV();
        __WFI();    
    }
}


static uint8_t m_buffer[256];
#define SCHED_EVT_RX_DATA            1
#define  DFU_SERIAL_CMD_JumpToMasterBoot        0x07

/**
 * @brief Process data received from serial port.
 * @param[in] p_event_data event type.
 * @param[in] event_size event size.
 * @return none
 */
static void sched_evt(void * p_event_data, uint16_t event_size)
{
    switch(*(uint8_t *)p_event_data)
    {
        case SCHED_EVT_RX_DATA:{        
            if(m_buffer[0] == 0xAA)
            {
                switch(m_buffer[1]){
                    case DFU_SERIAL_CMD_JumpToMasterBoot:{
                        if(m_buffer[2] == 0x01 && m_buffer[3] == 0x02 && m_buffer[4] == 0x03)
                        {
                            uint8_t cmd[] = {0xAA,DFU_SERIAL_CMD_JumpToMasterBoot,0};
                            serial_send_data(cmd, sizeof(cmd));
                            
                            if(ns_dfu_boot_force_usart_dfu() == false){
                                uint8_t cmd[] = {0xAA,DFU_SERIAL_CMD_JumpToMasterBoot,2};
                                serial_send_data(cmd, sizeof(cmd));                            
                            
                            }
                        }else
                        {
                            uint8_t cmd[] = {0xAA,DFU_SERIAL_CMD_JumpToMasterBoot,1};
                            serial_send_data(cmd, sizeof(cmd));                        
                        
                        }
                    }break;
                }
            }
        }break;
    }
}




void USART1_IRQHandler(void)
{
    static uint32_t index = 0;
    static uint8_t buffer[256];
    
    if(USART_GetFlagStatus(USART1, USART_FLAG_RXDNE) != RESET)
    {
        buffer[index] = USART_ReceiveData(USART1);
        
        if(buffer[0] == 0xAA)
            {
                index++;
                if(index >= 256)
                {
                    index = 0;    
                    memset(m_buffer,0,sizeof(m_buffer));
                    memcpy(m_buffer,buffer, 256);
                    
                    uint8_t event = SCHED_EVT_RX_DATA;
                    uint32_t    err_code = app_sched_event_put(&event ,sizeof(uint8_t),sched_evt);
                    ERROR_CHECK(err_code);
                }                            
            }        
        
        
    }    
}



static uint32_t serial_send_data(uint8_t *p_data, uint32_t length)
{
    static uint8_t cmd[256];
    memset(cmd,0,sizeof(cmd));
    memcpy(cmd,p_data,length);
    dfu_usart1_send(cmd,sizeof(cmd));
    
    return true;
}


