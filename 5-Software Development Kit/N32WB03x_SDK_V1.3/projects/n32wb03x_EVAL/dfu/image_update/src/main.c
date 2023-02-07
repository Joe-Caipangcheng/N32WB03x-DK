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
#include "n32wb03x.h"
#include "dfu_crc.h"
#include "ble_stack_common.h"
#include "ns_ble.h"
#include "prf.h"
#include "ns_dfu_boot.h"


/**
 * @brief Set the bootsetting to default value, so next time boot up will not goin to imageupdate again.
 * @param[in] none.
 * @return none
 */
static void bootsetting_reset(void)
{
    NS_Bootsetting_t m_ns_bootsetting;
    memcpy(&m_ns_bootsetting,&ns_bootsetting,sizeof(NS_Bootsetting_t));    
    Qflash_Init();
    if(ns_bootsetting.app1.crc == dfu_crc32((uint8_t *)((uint32_t *)ns_bootsetting.app1.start_address), ns_bootsetting.app1.size) )
    {
        m_ns_bootsetting.app1.activation = NS_BOOTSETTING_ACTIVATION_YES;
        m_ns_bootsetting.ImageUpdate.activation = NS_BOOTSETTING_ACTIVATION_NO;
    }
    else if(ns_bootsetting.app2.crc == dfu_crc32((uint8_t *)((uint32_t *)ns_bootsetting.app2.start_address), ns_bootsetting.app2.size) )
    {
        m_ns_bootsetting.app2.activation = NS_BOOTSETTING_ACTIVATION_YES;
        m_ns_bootsetting.ImageUpdate.activation = NS_BOOTSETTING_ACTIVATION_NO;
    }
    if(m_ns_bootsetting.ImageUpdate.activation == NS_BOOTSETTING_ACTIVATION_NO)
    {
        m_ns_bootsetting.crc = dfu_crc32((uint8_t *)&m_ns_bootsetting.crc + 4, sizeof(NS_Bootsetting_t) - 4);
        Qflash_Erase_Sector(NS_BOOTSETTING_START_ADDRESS);
        Qflash_Write(NS_BOOTSETTING_START_ADDRESS, (uint8_t *)&m_ns_bootsetting, sizeof(NS_Bootsetting_t));        
    }
}

int main(void)
{
    bootsetting_reset();
    
    app_init();
    prf_init(RWIP_INIT);

    while(1)
    {
        rwip_schedule();        
    }
}


