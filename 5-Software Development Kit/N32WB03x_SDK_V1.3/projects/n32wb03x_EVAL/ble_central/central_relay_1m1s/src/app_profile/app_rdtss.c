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
 * @file app_rdtss.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** 
 * @addtogroup APP
 * @{ 
 */

#include "rwip_config.h"     // SW configuration

#if (BLE_RDTSS_SERVER)

/* Includes ------------------------------------------------------------------*/
#include "co_utils.h"
#include "app_rdtss.h"                    // rdtss Application Module Definitions
#include "rdtss_task.h"                   // Device Information Profile Functions
#include "gapm_task.h"                   // GAP Manager Task API
#include "ns_ble.h"
#include "prf.h"
#include "ke_timer.h"

#include "stdio.h"
#include "rdtss.h" 
#include "app_fifo.h"
#include "app_rdtsc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

const uint8_t app_rdts_svc_uuid[16] = ATT_SERVICE_AM_SPEED_128;
const struct attm_desc_128 app_rdts_att_db[RDTSS_IDX_NB] =
{
    /* Service Declaration */
    [0] = {{0x00,0x28},                 PERM(RD, ENABLE),                                               0,                                          0       },

    /* Characteristic Declaration */
    [1] = {{0x03,0x28},                 PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE),                     0,                                          0       },
    /* Characteristic Value */
    [2] = {ATT_CHAR_AM_SPEED_WRITE_128, PERM(WRITE_COMMAND, ENABLE),                                     PERM(RI, ENABLE)| PERM_VAL(UUID_LEN, 0x02), 0x200   },
    /* Client Characteristic Configuration Descriptor */
    [3] = {{0x01,0x29},                 PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE),                     PERM(RI, ENABLE),                           20      },

    /* Characteristic Declaration */
    [4] = {{0x03,0x28},                 PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE),                     0,                                          0       },
    /* Characteristic Value */
    [5] = {ATT_CHAR_AM_SPEED_NTF_128,   PERM(NTF, ENABLE),                                              PERM(RI, ENABLE)| PERM_VAL(UUID_LEN, 0x02), 0x200   },   
    /* Client Characteristic Configuration Descriptor */
    [6] = {{0x02,0x29},                 PERM(RD, ENABLE) |PERM(WRITE_REQ, ENABLE),                      PERM(RI, ENABLE),                           20      },

    /* Characteristic Declaration */
    [7] = {{0x03,0x28},                 PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE),                     0,                                          0       },
    /* Characteristic Value */
    [8] = {ATT_CHAR_AM_SPEED_CNTL_POINT_128,   PERM(WRITE_COMMAND, ENABLE),                             PERM_VAL(UUID_LEN, 0x02),                   20      },

    /* Characteristic Declaration */
    [9] = {{0x03,0x28},                 PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE),                     0,                                          0       },
    /* Characteristic Value */
    [10] = {ATT_CHAR_AM_SPEED_CMD_CMP_NTF_128,   PERM(NTF, ENABLE),                                      PERM(RI, ENABLE)| PERM_VAL(UUID_LEN, 0x02), 0x200   },   
    /* Client Characteristic Configuration Descriptor */
    [11] = {{0x02,0x29},                 PERM(RD, ENABLE) |PERM(WRITE_REQ, ENABLE),                      PERM(RI, ENABLE),                           20      },


};


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/** 
 * @brief Create raw data transfer server profile database.
 * @return void
 */
void app_rdtss_add_rdts(void)
{
    struct rdtss_db_cfg *db_cfg;

    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
                                                             TASK_GAPM,
                                                             TASK_APP,
                                                             gapm_profile_task_add_cmd,
                                                             sizeof(struct rdtss_db_cfg));

    // Fill message
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = PERM(SVC_AUTH, NO_AUTH);   
    req->prf_task_id = TASK_ID_RDTSS;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    // Set parameters
    db_cfg = (struct rdtss_db_cfg *) req->param;
    // Attribute table. In case the handle offset needs to be saved
    db_cfg->att_tbl = &app_rdts_att_db[0];
    db_cfg->svc_uuid = &app_rdts_svc_uuid[0];
    db_cfg->max_nb_att = RDTSS_IDX_NB;

    // Send the message
    ke_msg_send(req);
    
    app_rdtss_init();
}

/**
 * @brief  rdtss value require indicate handler
 * @param   
 * @return 
 * @note   Note
 */
static int rdtss_value_req_ind_handler(ke_msg_id_t const msgid,
                                          struct rdtss_value_req_ind const *req_value,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    NS_LOG_DEBUG("%s\r\n",__func__);
    
    // Initialize length
    uint8_t len = 0;
    // Pointer to the data
    uint8_t *data = NULL;
    
    len = APP_RDTSS_MANUFACTURER_NAME_LEN;
    data = (uint8_t *)APP_RDTSS_MANUFACTURER_NAME;
    
    
    // Allocate confirmation to send the value
    struct rdtss_value_req_rsp *rsp_value = KE_MSG_ALLOC_DYN(RDTSS_VALUE_REQ_RSP,
                                                                src_id, dest_id,
                                                                rdtss_value_req_rsp,
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
static int rdtss_val_write_ind_handler(ke_msg_id_t const msgid,
                                          struct rdtss_val_write_ind const *ind_value,
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
		uint16_t conidx_m = ns_ble_get_conidx_by_role(ROLE_MASTER);
    
    switch (handle)
    {
        case RDTSS_IDX_NTF_CFG:
            
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
        case RDTSS_IDX_WRITE_VAL:
            //ble data receive
            NS_LOG_INFO("Data from connected central\r\n");
            user_send_ble_data(ind_value->value,ind_value->length);
            break;
        
        case RDTSS_IDX_CNTL_POINT_VAL:
            {
                
                switch(ind_value->value[0])
                {
                    case CNTL_POINT_OP_CONN:
                    {
                        //data strructure: || addr_type || addr ||
                        //                 ||    1B     ||  6B  ||
                        
                        uint8_t i;
                        
                        //check addr type
                        if(ind_value->value[1] > 1)
                        {
                            NS_LOG_INFO("ADDR type error: %x\r\n", ind_value->value[1]);
                        }
                        else
                        {
													if(conidx_m != GAP_INVALID_CONIDX)
													{
														NS_LOG_INFO("Master connetion has been established\r\n");
														rdtss_notify_ctrl_point_cmd_cmp(CNTL_POINT_OP_CONN, STATUS_ERR_MASTER_HAS_CONNECTED, NULL, 0);
													}
													else
													{
														     NS_LOG_INFO("Start connecting peripheral, index: %x\r\n", i);
                                NS_LOG_INFO("Target ADDR: ");
                                for(uint8_t k=0; k<6; k++)
                                {
                                    NS_LOG_INFO(" %x ", ind_value->value[k+2]);
                                }
                                NS_LOG_INFO("\r\n");
                                
                                ns_ble_stop_scan();
                                NS_LOG_INFO("Stop SCAN\r\n");
                                
                                ns_ble_start_init((uint8_t *)&(ind_value->value[2]), ind_value->value[1]);
													}
                        }
                        
                    }break;
                    
                    case CNTL_POINT_OP_DISCONN:
                    {
                        NS_LOG_INFO("IDX = %x \r\n", conidx_m);
                        
                        if(conidx_m != GAP_INVALID_CONIDX)
                        {
													 ns_ble_set_active_connection(conidx_m);
                           ns_ble_disconnect();
                        }
                        else
                        {
                            rdtss_notify_ctrl_point_cmd_cmp(CNTL_POINT_OP_DISCONN, STATUS_ERR_NOT_CONNECTED, NULL, 0);
                        }
                    }break;
                    
                    default:break;
                }
            }
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
static int rdtss_val_ntf_cfm_handler(ke_msg_id_t const msgid,
                                          struct rdtss_val_ntf_cfm const *cfm_value,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    NS_LOG_DEBUG("%s,ntf cfm handle = %x, status = %x\r\n",__func__, cfm_value->handle, cfm_value->status);
    
    pop_matser_rx_to_ble();
    
    return (KE_MSG_CONSUMED);
}

/**
 * @brief  rdtss send notify
 * @param   
 * @return 
 * @note   
 */
void rdtss_send_notify(uint8_t *data, uint16_t length)
{
    if(ns_ble_slave_connection_num())
    {
        struct rdtss_val_ntf_ind_req *req = KE_MSG_ALLOC_DYN(RDTSS_VAL_NTF_REQ,
                                                              prf_get_task_from_id(TASK_ID_RDTSS),
                                                              TASK_APP,
                                                              rdtss_val_ntf_ind_req,
                                                              length);

        req->conidx = ns_ble_get_conidx_by_role(ROLE_SLAVE);
        req->notification = true;
        req->handle = RDTSS_IDX_NTF_VAL;
        req->length = length;
        memcpy(&req->value[0], data, length);

        NS_LOG_DEBUG("%x", req->value[0]);
        
        ke_msg_send(req);
    }
    else
    {
        NS_LOG_INFO("Not connected with central device\r\n");
    }
}

void rdtss_notify_ctrl_point_cmd_cmp(uint8_t cmd, uint8_t status, uint8_t *data, uint16_t length)
{
    if(ns_ble_slave_connection_num())
    {
        struct rdtss_val_ntf_ind_req *req = KE_MSG_ALLOC_DYN(RDTSS_VAL_NTF_REQ,
                                                              prf_get_task_from_id(TASK_ID_RDTSS),
                                                              TASK_APP,
                                                              rdtss_val_ntf_ind_req,
                                                              length + 4);

        req->conidx = ns_ble_get_conidx_by_role(ROLE_SLAVE);
        req->notification = true;
        req->handle = RDTSS_IDX_CMD_CMP_NTF_VAL;
        
        req->length = length + 4;
        
        req->value[0] = CNTL_POINT_CMD_CMP;
        req->value[1] = cmd;
        req->value[2] = status;
        req->value[3] = length;
        
        if(data != NULL)
        {
            memcpy(&req->value[4], data, length);
        }
        
        ke_msg_send(req);
    }
    else
    {
        NS_LOG_INFO("Not connected with central device\r\n");
    }
}

/// Default State handlers definition
const struct ke_msg_handler app_rdtss_msg_handler_list[] =
{
    {RDTSS_VALUE_REQ_IND,                    (ke_msg_func_t)rdtss_value_req_ind_handler},
    {RDTSS_VAL_WRITE_IND,                    (ke_msg_func_t)rdtss_val_write_ind_handler},
    {RDTSS_VAL_NTF_CFM,                      (ke_msg_func_t)rdtss_val_ntf_cfm_handler},
    
};

const struct app_subtask_handlers app_rdtss_handlers = APP_HANDLERS(app_rdtss);


void app_rdtss_init(void)
{
    //register application subtask to app task
    struct prf_task_t prf;
    prf.prf_task_id = TASK_ID_RDTSS;
    prf.prf_task_handler = &app_rdtss_handlers;
    ns_ble_prf_task_register(&prf);
    
    //register get itf function to prf.c
    struct prf_get_func_t get_func;
    get_func.task_id = TASK_ID_RDTSS;
    get_func.prf_itf_get_func = rdtss_prf_itf_get;
    prf_get_itf_func_register(&get_func);
}

#endif //BLE_RDTSS_SERVER
