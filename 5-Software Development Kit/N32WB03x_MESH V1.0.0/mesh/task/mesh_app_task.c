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
 * @file mesh_app_task.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "mesh_app_task.h"
#include "n32wb03x.h"
#include "gapm_task.h"
#include "rwip.h"
#include "co_bt_defines.h"
#include "gapc_task.h" 
#include "ke_task.h" 
#include "co_utils.h" 
#include "stdio.h" 
#include "scanner.h" 

/** 
 * @brief Start to scan.
 *
 * @param[in] msgid     Id of the message.
 * @param[in] param   	Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the task instance (TASK_MESH).
 * @param[in] src_id    ID of the task instance.
 *
 * @return If the message was consumed or not. 
 */
static int mesh_task_start_scan_handler(ke_msg_id_t const msgid, void const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    scanner_radio_start();
    return (KE_MSG_CONSUMED);
}
/** 
 * @brief Stop scanning.
 *
 * @param[in] msgid     Id of the message.
 * @param[in] param   	Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the task instance (TASK_MESH).
 * @param[in] src_id    ID of the task instance.
 *
 * @return If the message was consumed or not. 
 */
static int mesh_task_stop_scan_handler(ke_msg_id_t const msgid, void const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    scanner_radio_stop();
    return (KE_MSG_CONSUMED);
}


extern int mesh_task_button_timer_handler(ke_msg_id_t const msgid,void *p_param,ke_task_id_t const dest_id,ke_task_id_t const src_id,enum ke_msg_status_tag *msg_ret);
extern int mesh_task_flash_evt_handler(ke_msg_id_t const msgid,void *p_param,ke_task_id_t const dest_id,ke_task_id_t const src_id,enum ke_msg_status_tag *msg_ret);
extern int mesh_task_flash_end_evt_handler(ke_msg_id_t const msgid,void *p_param,ke_task_id_t const dest_id,ke_task_id_t const src_id,enum ke_msg_status_tag *msg_ret);
extern int mesh_task_bearer_evt_handler(ke_msg_id_t const msgid,void *p_param,ke_task_id_t const dest_id,ke_task_id_t const src_id,enum ke_msg_status_tag *msg_ret);
extern int mesh_task_gatt_reset_handler(ke_msg_id_t const msgid,void *p_param,ke_task_id_t const dest_id,ke_task_id_t const src_id,enum ke_msg_status_tag *msg_ret);
extern int mesh_task_wdt_handler(ke_msg_id_t const msgid,void *p_param,ke_task_id_t const dest_id,ke_task_id_t const src_id,enum ke_msg_status_tag *msg_ret);
extern int mesh_task_schedule_timer_handler(ke_msg_id_t const msgid,void *p_param,ke_task_id_t const dest_id,ke_task_id_t const src_id,enum ke_msg_status_tag *msg_ret);
extern int mesh_task_ad_report_proc_handler(ke_msg_id_t const msgid,void *p_param,ke_task_id_t const dest_id,ke_task_id_t const src_id,enum ke_msg_status_tag *msg_ret);
extern int mesh_task_broadcast_timeout_handler(ke_msg_id_t const msgid,void *p_param,ke_task_id_t const dest_id,ke_task_id_t const src_id,enum ke_msg_status_tag *msg_ret);
extern int mesh_task_timer_sched_timeout_handler(ke_msg_id_t const msgid,void *p_param,ke_task_id_t const dest_id,ke_task_id_t const src_id,enum ke_msg_status_tag *msg_ret);


/* Mesh task related events handlers. */
KE_MSG_HANDLER_TAB(mesh)
{
    {MESH_TASK_BUTTON_TIMER,        (ke_msg_func_t)mesh_task_button_timer_handler},

    
    {MESH_TASK_FLASH_EVT,           (ke_msg_func_t)mesh_task_flash_evt_handler},
    {MESH_TASK_FLASH_END_EVT,       (ke_msg_func_t)mesh_task_flash_end_evt_handler},
    {MESH_TASK_BEARER_EVENT,        (ke_msg_func_t)mesh_task_bearer_evt_handler},
    {MESH_TASK_START_SCAN,          (ke_msg_func_t)mesh_task_start_scan_handler},
    {MESH_TASK_STOP_SCAN,           (ke_msg_func_t)mesh_task_stop_scan_handler},
    {MESH_TASK_GATT_RESET,          (ke_msg_func_t)mesh_task_gatt_reset_handler},
    {MESH_TASK_WDT,                 (ke_msg_func_t)mesh_task_wdt_handler},
    {MESH_TASK_SCHEDULE_TIMER,      (ke_msg_func_t)mesh_task_schedule_timer_handler},    
    {MESH_TASK_AD_REPORT_PROC,      (ke_msg_func_t)mesh_task_ad_report_proc_handler},
    {MESH_TASK_BROADCAST_TIMEOUT,   (ke_msg_func_t)mesh_task_broadcast_timeout_handler},
    {MESH_TASK_TIMER_SCHED_TIMEOUT, (ke_msg_func_t)mesh_task_timer_sched_timeout_handler},

};
/* Defines the place holder for the states of all the task instances. */
ke_state_t mesh_state[1];
/* Mesh task descriptor */
const struct ke_task_desc TASK_DESC_MESH_M = {mesh_msg_handler_tab, mesh_state, 1, ARRAY_LEN(mesh_msg_handler_tab)};









