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
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** @addtogroup 
 * @{
 */

 /* Includes ------------------------------------------------------------------*/

#include "dfu_led.h"
#include "n32wb03x.h"
#include "dfu_delay.h"
#include "rwip.h"
#include "ns_ble.h"
#include "app_ble.h"
#include "ns_sleep.h"
#include "ns_delay.h"
#include "ns_log.h"
#include "ns_dfu_boot.h"


/**
 * @brief  main function
 * @param   
 * @return 
 * @note   Note
 */
int main(void)
{

    NS_LOG_INIT();    
    
    if(CURRENT_APP_START_ADDRESS == NS_APP1_START_ADDRESS){
        NS_LOG_INFO("application 1 start new ...\r\n");
    }else if(CURRENT_APP_START_ADDRESS == NS_APP2_START_ADDRESS){
        NS_LOG_INFO("application 2 start new ...\r\n");
    }    
    
    dfu_leds_config();
    
    for(uint8_t i=0;i<10;i++)
    {
        dfu_led_toggle(LED1_GPIO_PORT, LED_GPIO1_PIN);
        if(CURRENT_APP_START_ADDRESS == NS_APP1_START_ADDRESS){
            dfu_delay_ms(100);
        }else if(CURRENT_APP_START_ADDRESS == NS_APP2_START_ADDRESS){
            dfu_delay_ms(500);
        }        
        dfu_led_toggle(LED2_GPIO_PORT, LED_GPIO2_PIN);
    }
    dfu_led_off(LED1_GPIO_PORT, LED_GPIO1_PIN);
    dfu_led_off(LED2_GPIO_PORT, LED_GPIO2_PIN);    

	app_ble_init();

    while(1)
    {
        rwip_schedule();    
        ns_sleep();                  
    }
}


