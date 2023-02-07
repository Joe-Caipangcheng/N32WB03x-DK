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
 * @file mesh_app_task.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef __MESH_APP_TASK_H__
#define __MESH_APP_TASK_H__





#include "rwip_config.h"   
#include <stdint.h>         // Standard Integer
#include "rwip_task.h"      // Task definitions
#include "ke_task.h"        // Kernel Task
#include "ke_timer.h" 


/* Mesh tasks id */
enum mesh_msg_id
{
    MESH_DUMMY_MSG = TASK_FIRST_MSG(TASK_ID_MESH),

    MESH_TASK_BUTTON_TIMER,
    MESH_TASK_FLASH_EVT,
    MESH_TASK_FLASH_END_EVT,
    MESH_TASK_BEARER_EVENT,
    MESH_TASK_START_SCAN,
    MESH_TASK_STOP_SCAN,
    MESH_TASK_GATT_RESET,
    MESH_TASK_WDT,    
    MESH_TASK_SCHEDULE_TIMER,
    MESH_TASK_AD_REPORT_PROC,  
    MESH_TASK_BROADCAST_TIMEOUT,
    MESH_TASK_TIMER_SCHED_TIMEOUT,
};





#endif //__MESH_APP_TASK_H__

