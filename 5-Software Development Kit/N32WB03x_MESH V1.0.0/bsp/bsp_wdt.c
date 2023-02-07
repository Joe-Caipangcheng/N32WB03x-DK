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
 * @file bsp_wdt.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** @addtogroup 
 * @{
 */
#include "bsp_wdt.h"
#include "n32wb03x.h"
#include "ke_timer.h"
#include "stdio.h"
#include "mesh_app_task.h"
#include "bsp_led.h"


/*Independent watch dog init*/
void bsp_wdt_init(void)
{
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, ENABLE);
	DBG_ConfigPeriph(DBG_IWDG_STOP, ENABLE);    
    if (RCC_GetFlagStatus(RCC_CTRLSTS_FLAG_IWDGRSTF) != RESET)
    {
		printf("reset by WDG\r\n"); 
        RCC_ClrFlag();
    } 
    IWDG_WriteConfig(IWDG_WRITE_ENABLE);
    IWDG_SetPrescalerDiv(IWDG_PRESCALER_DIV256);
    IWDG_CntReload(256*10); //Configure watch dog to 10 seconds timeout and reset system.
    IWDG_ReloadKey();
    
}

/*Feed watch dog timer handler*/
int mesh_task_wdt_handler(ke_msg_id_t const msgid, void const *p_param,ke_task_id_t const dest_id,ke_task_id_t const src_id)
{   
    IWDG_ReloadKey();
    ke_timer_set(MESH_TASK_WDT, TASK_MESH, 2000); 
}



/*Watch dog start*/
void bsp_wdt_start(void)
{
    IWDG_Enable();
    IWDG_ReloadKey();
    //Start a 2 seconds timer to feed the watchdog
    ke_timer_set(MESH_TASK_WDT, TASK_MESH, 2000); 
}


/*Watch dog feed*/
void bsp_wdt_feed(void)
{
    IWDG_ReloadKey();

}


