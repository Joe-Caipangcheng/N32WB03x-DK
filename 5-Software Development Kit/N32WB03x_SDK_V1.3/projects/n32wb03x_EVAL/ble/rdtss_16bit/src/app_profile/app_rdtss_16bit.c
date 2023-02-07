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
 * @file app_rdtss_16bit.c
 * @author Nations Firmware Team
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** 
 * @addtogroup APP
 * @{ 
 */

#include "rwip_config.h"     // SW configuration
#include "stdio.h"
#if (BLE_RDTSS_16BIT_SERVER)

/* Includes ------------------------------------------------------------------*/
#include "rdtss_16bit.h"
#include "app_rdtss_16bit.h" 
#include "rdtss_16bit_task.h" 
#include "gapm_task.h"                  
#include "ns_ble.h"
#include "prf.h"
#include "ke_timer.h"


#include "app_usart.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

const uint16_t app_rdts_svc_uuid = ATT_SERVICE_AM_SPEED_16;

const struct attm_desc app_rdts_att_db[RDTSS_16BIT_IDX_NB] =
{
    /* Service Declaration */
    [0] = {ATT_DECL_PRIMARY_SERVICE, PERM(RD, ENABLE),                                              0,                                          0       },

    /* Characteristic Declaration */
    [1] = {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE),                     0,                                          0       },
    /* Characteristic Value */
    [2] = {ATT_CHAR_AM_SPEED_WRITE_16, PERM(WRITE_COMMAND, ENABLE),                                 PERM(RI, ENABLE)| PERM_VAL(UUID_LEN, 0), 0x200   },
    /* Client Characteristic Configuration Descriptor */
    [3] = {ATT_DESC_CHAR_USER_DESCRIPTION, PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE),              PERM(RI, ENABLE),                           20      },

    /* Characteristic Declaration */
    [4] = {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE),                     0,                                          0       },
    /* Characteristic Value */
    [5] = {ATT_CHAR_AM_SPEED_NTF_16, PERM(NTF, ENABLE),                                              PERM(RI, ENABLE)| PERM_VAL(UUID_LEN, 0), 0x200   },   
    /* Client Characteristic Configuration Descriptor */
    [6] = {ATT_DESC_CLIENT_CHAR_CFG, PERM(RD, ENABLE) |PERM(WRITE_REQ, ENABLE),                      PERM(RI, ENABLE),                           20      },
};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/** 
 * @brief Create raw data transfer server profile database.
 * @return void
 */
void app_rdtss_16bit_add_rdtss_16bit(void)
{
    struct rdtss_16bit_db_cfg *db_cfg;

    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
                                                             TASK_GAPM,
                                                             TASK_APP,
                                                             gapm_profile_task_add_cmd,
                                                             sizeof(struct rdtss_16bit_db_cfg));
    NS_LOG_DEBUG("%s\r\n",__func__);
    // Fill message
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = PERM(SVC_AUTH, NO_AUTH);
    req->prf_task_id = TASK_ID_RDTSS_16BIT;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    // Set parameters
    db_cfg = (struct rdtss_16bit_db_cfg *) req->param;
    // Attribute table. In case the handle offset needs to be saved
    db_cfg->att_tbl     = &app_rdts_att_db[0];
    db_cfg->svc_uuid    = &app_rdts_svc_uuid;
    db_cfg->max_nb_att  = RDTSS_16BIT_IDX_NB;
    // Send the message
    ke_msg_send(req);
    
    app_rdtss_16bit_init();
}

/**
 * @brief  rdtss value require indicate handler
 * @param   
 * @return 
 * @note   Note
 */
static int rdtss_16bit_value_req_ind_handler(ke_msg_id_t const msgid,
                                          struct rdtss_16bit_value_req_ind const *req_value,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    NS_LOG_DEBUG("%s\r\n",__func__);
    
    // Initialize length
    uint8_t len = 0;
    // Pointer to the data
    uint8_t *data = NULL;
    
    len = APP_RDTSS_16BIT_MANUFACTURER_NAME_LEN;
    data = (uint8_t *)APP_RDTSS_16BIT_MANUFACTURER_NAME;
    
    
    // Allocate confirmation to send the value
    struct rdtss_16bit_value_req_rsp *rsp_value = KE_MSG_ALLOC_DYN(RDTSS_16BIT_VALUE_REQ_RSP,
                                                                src_id, dest_id,
                                                                rdtss_16bit_value_req_rsp,
                                                                len);

    rsp_value->length = len;
    rsp_value->att_idx = req_value->att_idx;
    if (len)
    {
        // Copy data
        memcpy(&rsp_value->value, data, len);
    }
    // Send message
    ke_msg_send(rsp_value);
    
    return (KE_MSG_CONSUMED);
}

/**
 * @brief  cuusts write indicate handler
 * @param   
 * @return  
 * @note   
 */
static int rdtss_16bit_val_write_ind_handler(ke_msg_id_t const msgid,
                                          struct rdtss_16bit_val_write_ind const *ind_value,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    NS_LOG_DEBUG("%s,write handle = %x,length = %x\r\n",__func__ ,ind_value->handle, ind_value->length);
    
    for(uint16_t i=0; i<ind_value->length; i++)
    {
        NS_LOG_DEBUG("%x ",ind_value->value[i]);
    }
    NS_LOG_DEBUG("\r\n");
    
    uint16_t handle = ind_value->handle;
    uint16_t length = ind_value->length;
    
    switch (handle)
    {
        case RDTSS_16BIT_IDX_NTF_CFG:
            
            NS_LOG_DEBUG("RDTSS_16BIT_IDX_NTF_CFG\r\n");
        
            if(length == 2)
            {
                uint16_t cfg_value = ind_value->value[0] + ind_value->value[1];
                
                if(cfg_value == PRF_CLI_START_NTF)
                {
                    //enabled notify 
                }
                else if(cfg_value == PRF_CLI_STOP_NTFIND)
                {
                }
            }
            
            break;
        case RDTSS_16BIT_IDX_WRITE_VAL:
            NS_LOG_DEBUG("RDTSS_16BIT_IDX_WRITE_VAL\r\n");
            app_usart_tx_fifo_enter(ind_value->value,ind_value->length);
          break;
        default:
            break;
        
    }
    
    return (KE_MSG_CONSUMED);
}

/**
 * @brief  Functions
 * @param   
 * @return 
 * @note   Note
 */
static int rdtss_16bit_val_ntf_cfm_handler(ke_msg_id_t const msgid,
                                          struct rdtss_16bit_val_ntf_cfm const *cfm_value,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    NS_LOG_DEBUG("%s,ntf cfm handle = %x, status = %x\r\n",__func__, cfm_value->handle, cfm_value->status);
    
    usart_forward_to_ble_loop();
    
    return (KE_MSG_CONSUMED);
}

/**
 * @brief  rdtss_16bit send notify
 * @param   
 * @return 
 * @note   
 */
void rdtss_16bit_send_notify(uint8_t *data, uint16_t length)
{
    struct rdtss_16bit_val_ntf_ind_req *req = KE_MSG_ALLOC_DYN(RDTSS_16BIT_VAL_NTF_REQ,
                                                          prf_get_task_from_id(TASK_ID_RDTSS_16BIT),
                                                          TASK_APP,
                                                          rdtss_16bit_val_ntf_ind_req,
                                                          length);

    req->conidx = app_env.conidx;
    req->notification = true;
    req->handle = RDTSS_16BIT_IDX_NTF_VAL;
    req->length = length;
    memcpy(&req->value[0], data, length);

    ke_msg_send(req);
}


/// Default State handlers definition
const struct ke_msg_handler app_rdtss_16bit_msg_handler_list[] =
{
    {RDTSS_16BIT_VALUE_REQ_IND,                    (ke_msg_func_t)rdtss_16bit_value_req_ind_handler},
    {RDTSS_16BIT_VAL_WRITE_IND,                    (ke_msg_func_t)rdtss_16bit_val_write_ind_handler},
    {RDTSS_16BIT_VAL_NTF_CFM,                      (ke_msg_func_t)rdtss_16bit_val_ntf_cfm_handler},
    
};

const struct app_subtask_handlers app_rdtss_16bit_handlers = APP_HANDLERS(app_rdtss_16bit);

void app_rdtss_16bit_init(void)
{
    //register application subtask to app task
    struct prf_task_t prf;
    prf.prf_task_id = TASK_ID_RDTSS_16BIT;
    prf.prf_task_handler = &app_rdtss_16bit_handlers;
    ns_ble_prf_task_register(&prf);
    
    //register get itf function to prf.c
    struct prf_get_func_t get_func;
    get_func.task_id = TASK_ID_RDTSS_16BIT;
    get_func.prf_itf_get_func = rdtss_16bit_prf_itf_get;
    prf_get_itf_func_register(&get_func);
    
}

#endif //BLE_RDTSS_16BIT_SERVER
