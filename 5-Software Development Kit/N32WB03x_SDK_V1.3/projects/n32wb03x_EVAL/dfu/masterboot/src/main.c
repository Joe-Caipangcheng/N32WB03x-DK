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
#include "n32wb03x.h"
#include "dfu_led.h"
#include "ns_dfu_boot.h"
#include "dfu_crc.h"
#include "ns_scheduler.h"
#include "ns_dfu_serial.h"


/**
 * @brief Loop up the bootsetting and decide which bank to jump to , or stay in masterboot for usart image update.
 * @param[in] none.
 * @return none
 */
static void masterboot(void)
{
    if(ns_bootsetting.crc == dfu_crc32((uint8_t *)&ns_bootsetting.crc + 4, sizeof(NS_Bootsetting_t) - 4))
    {
        if(ns_bootsetting.master_boot_force_update != NS_BOOTSETTING_MASTER_BOOT_FORCE_UPDATE_YES)
        {
            if(ns_bootsetting.app1.activation == NS_BOOTSETTING_ACTIVATION_YES && ns_bootsetting.app1.start_address == NS_APP1_START_ADDRESS)
            {
                if(ns_bootsetting.app1.crc == dfu_crc32((uint8_t *)((uint32_t *)ns_bootsetting.app1.start_address), ns_bootsetting.app1.size))
                {
                    ns_dfu_boot_jump(ns_bootsetting.app1.start_address);
                }
            }    
            if(ns_bootsetting.app2.activation == NS_BOOTSETTING_ACTIVATION_YES && ns_bootsetting.app2.start_address == NS_APP2_START_ADDRESS)
            {
                if(ns_bootsetting.app2.crc == dfu_crc32((uint8_t *)((uint32_t *)ns_bootsetting.app2.start_address), ns_bootsetting.app2.size))
                {
                    ns_dfu_boot_jump(ns_bootsetting.app2.start_address);
                }
            }    
            if(ns_bootsetting.ImageUpdate.activation == NS_BOOTSETTING_ACTIVATION_YES && ns_bootsetting.ImageUpdate.start_address == NS_IMAGE_UPDATE_START_ADDRESS)
            {
                if(ns_bootsetting.ImageUpdate.crc == dfu_crc32((uint8_t *)((uint32_t *)ns_bootsetting.ImageUpdate.start_address), ns_bootsetting.ImageUpdate.size))
                {
                    ns_dfu_boot_jump(ns_bootsetting.ImageUpdate.start_address);
                }
            }

            if(ns_bootsetting.app1.crc == dfu_crc32((uint8_t *)((uint32_t *)ns_bootsetting.app1.start_address), ns_bootsetting.app1.size))
            {
                ns_dfu_boot_jump(ns_bootsetting.app1.start_address);
            }        
            if(ns_bootsetting.app2.crc == dfu_crc32((uint8_t *)((uint32_t *)ns_bootsetting.app2.start_address), ns_bootsetting.app2.size))
            {
                ns_dfu_boot_jump(ns_bootsetting.app2.start_address);
            }            
            if(ns_bootsetting.ImageUpdate.crc == dfu_crc32((uint8_t *)((uint32_t *)ns_bootsetting.ImageUpdate.start_address), ns_bootsetting.ImageUpdate.size))
            {
                ns_dfu_boot_jump(ns_bootsetting.ImageUpdate.start_address);
            }            
        }
    }
}
/**
 * @brief  main function
 * @param   
 * @return 
 * @note   Note
 */
int main(void)
{
    masterboot();
    
    dfu_leds_config();
    dfu_led_on(LED1_GPIO_PORT, LED_GPIO1_PIN);
    dfu_led_on(LED2_GPIO_PORT, LED_GPIO2_PIN);


    NS_SCHED_INIT(256, 16);    
    ns_dfu_serial_init();
    
    while(1)
    {
        app_sched_execute();
        __WFE();
        __SEV();
        __WFE();    
    }
}





