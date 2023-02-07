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
 * @file app_hrps.c
 * @author Nations Firmware Team
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** 
 * @addtogroup APP
 * @{ 
 */

#include "rwip_config.h"     // SW configuration

#if (BLE_APP_HRPS)
/* Includes ------------------------------------------------------------------*/
#include "ns_ble.h"                     // Application Manager Definitions
#include "app_hrps.h"                // Device Information Service Application Definitions
#include "hrps_task.h"               // Device Information Profile Functions
#include "prf_types.h"               // Profile Common Types Definitions
#include "ke_task.h"                 // Kernel
#include "gapm_task.h"               // GAP Manager Task API
#include <string.h>
#include "co_utils.h"
#include "hrps.h"
#include "stdio.h"
#include "ns_timer.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/// Heart Rate Application environment
struct app_hrps_env_tag app_hrps_env;

uint16_t heart_rate_timer_id;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
 * @brief  hrps server Initialize
 * @param  
 * @return 
 * @note   
 */
void app_hrps_init(void)
{
    app_hrps_env.hrps_value = 35;
    
    //register application subtask to app task
    struct prf_task_t prf;
    prf.prf_task_id = TASK_ID_HRPS;
    prf.prf_task_handler = &app_hrps_handlers;
    ns_ble_prf_task_register(&prf);
    
    //register get itf function to prf.c
    struct prf_get_func_t get_func;
    get_func.task_id = TASK_ID_HRPS;
    get_func.prf_itf_get_func = hrps_prf_itf_get;
    prf_get_itf_func_register(&get_func);
}

/**
 * @brief  add hrps server
 * @param  
 * @return 
 * @note   
 */
void app_hrps_add_hrps(void)
{
    struct hrps_db_cfg* db_cfg;
    // Allocate the DISS_CREATE_DB_REQ
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
                                                  TASK_GAPM, TASK_APP,
                                                  gapm_profile_task_add_cmd, sizeof(struct hrps_db_cfg));
    // Fill message
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl =  PERM(SVC_AUTH, NO_AUTH);

    req->prf_task_id = TASK_ID_HRPS;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    // Set parameters
    db_cfg = (struct hrps_db_cfg* ) req->param;
    db_cfg->features = APP_HRPS_FEATURES;

    // Send the message
    ke_msg_send(req);
    
    app_hrps_init();
}

/**
 * @brief  enable heart rate profile
 * @param  
 * @return 
 * @note   
 */
void app_hrps_enable_prf(uint8_t conidx, uint8_t cfg)
{
    app_hrps_env.conidx = conidx;

    // Allocate the message
    struct hrps_enable_req * req = KE_MSG_ALLOC(HRPS_ENABLE_REQ,
                                                prf_get_task_from_id(TASK_ID_HRPS),
                                                TASK_APP,
                                                hrps_enable_req);

    // Fill in the parameter structure
    req->conidx             = conidx;

    // NTF status
    req->hr_meas_ntf        = cfg;

    // Send the message
    ke_msg_send(req);
}

/**
 * @brief  send heart rate value 
 * @param  
 * @return 
 * @note   
 */
void app_heart_rate_send(uint16_t heart_rate)
{
    // Allocate the message
    struct hrps_meas_send_cmd * cmd = KE_MSG_ALLOC(HRPS_MEAS_SEND_CMD,
                                                        prf_get_task_from_id(TASK_ID_HRPS),
                                                        TASK_APP,
                                                        hrps_meas_send_cmd);

    // Fill in the parameter structure
    cmd->meas_val.heart_rate = heart_rate;

    // Send the message
    ke_msg_send(cmd);
}

/**
 * @brief heart rate send timeout handler
 * @param  
 * @return 
 * @note   
 */
void app_heart_rate_timeout_handler(void)
{
    app_hrps_env.hrps_value ++;
    
    if(app_hrps_env.hrps_value >= 180)
    {
        app_hrps_env.hrps_value = 35;
    }
    
    app_heart_rate_send(app_hrps_env.hrps_value);
    
    ns_timer_cancel(heart_rate_timer_id);
    
    heart_rate_timer_id = ns_timer_create(HEART_RATE_SEND_DELAY,app_heart_rate_timeout_handler);
}

/**
 * @brief  HRPS config indicate handler
 * @param  
 * @return 
 * @note   
 */
static int hrps_cfg_indntf_ind_handler(ke_msg_id_t const msgid,
                                          struct hrps_cfg_indntf_ind const *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    if( (param->cfg_val & HRPS_CFG_NTF_EN) != 0)
    {
        app_hrps_enable_prf(0, PRF_CLI_START_NTF);
        heart_rate_timer_id = ns_timer_create(HEART_RATE_SEND_DELAY,app_heart_rate_timeout_handler);
    }
    else
    {
        app_hrps_enable_prf(0, PRF_CLI_STOP_NTFIND);
        ns_timer_cancel(heart_rate_timer_id);
    }
    
    return KE_MSG_CONSUMED;
}


/* Public variables ---------------------------------------------------------*/

/// Default State handlers definition
const struct ke_msg_handler app_hrps_msg_handler_list[] =
{
    {HRPS_CFG_INDNTF_IND,     (ke_msg_func_t)hrps_cfg_indntf_ind_handler},
};

const struct app_subtask_handlers app_hrps_handlers = APP_HANDLERS(app_hrps);


#endif //BLE_APP_HRPS

/// @} APP
