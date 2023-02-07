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
 * @file app.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** 
 * @addtogroup APP
 * @{ 
 */

#include "app.h"
#include <string.h>
#include <stdio.h>
#include "app_task.h"
#include "gapm_task.h"
#include "rwip.h"
#include "co_bt_defines.h"
#include "gapc_task.h" 
#include "ke_task.h" 
#include "app_task.h"
#include "mesh_gatt.h"




/*BLE stack reset*/
static void reset(void)
{
    struct gapm_reset_cmd *p_cmd = KE_MSG_ALLOC(GAPM_RESET_CMD,TASK_GAPM, TASK_APP,gapm_reset_cmd);
    p_cmd->operation = GAPM_RESET;
    ke_msg_send(p_cmd);
    rwip_schedule();
}
/*BLE device config*/
static void config(void)
{
    struct gapm_set_dev_config_cmd* p_cmd = KE_MSG_ALLOC(GAPM_SET_DEV_CONFIG_CMD,TASK_GAPM, TASK_APP,gapm_set_dev_config_cmd);
    p_cmd->operation = GAPM_SET_DEV_CONFIG;				
    p_cmd->role = GAP_ROLE_ALL;
    p_cmd->pairing_mode = GAPM_PAIRING_DISABLE;
    p_cmd->max_mtu = 69+60; 
    p_cmd->sugg_max_tx_time   = 2120;
    p_cmd->sugg_max_tx_octets = 73+60; 	
    p_cmd->privacy_cfg = 0;
    memset((void *)&p_cmd->irk.key[0], 0x00, KEY_LEN);
    ke_msg_send(p_cmd);
    rwip_schedule();
}
/*App init*/
void app_init(void)
{
    ke_task_create(TASK_APP, &TASK_DESC_APP_M);
    reset();
    config();

}

/*BLE connection parameter update*/
void app_update_param(uint8_t con_idx, struct app_conn_param *p_conn_param)
{
    struct gapc_param_update_cmd *p_cmd = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CMD,
                                                     KE_BUILD_ID(TASK_GAPC, con_idx), TASK_APP,
                                                     gapc_param_update_cmd);
    p_cmd->operation  = GAPC_UPDATE_PARAMS;
    p_cmd->intv_min   = p_conn_param->intv_min;
    p_cmd->intv_max   = p_conn_param->intv_max;
    p_cmd->latency    = p_conn_param->latency;
    p_cmd->time_out   = p_conn_param->time_out;
    p_cmd->ce_len_min = 0xFFFF;
    p_cmd->ce_len_max = 0xFFFF;
    ke_msg_send(p_cmd);
}





