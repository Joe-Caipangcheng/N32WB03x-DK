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
 * @file app_rdtsc.c
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

#if (BLE_RDTS_CLIENT)

/* Includes ------------------------------------------------------------------*/
#include "co_utils.h"
#include "app_rdtsc.h"
#include "rdts_client_task.h"
#include "gapm_task.h"
#include "ns_ble.h"
#include "prf.h"
#include "ke_timer.h"
#include "stdio.h"
#include "ns_ble_task.h"
#include "app_usart.h"
#include "string.h"
#include "prf_utils_128.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define RDTS_SERVICE_UUID    {0x01, 0x10, 0x2E, 0xC7, 0x8a, 0x0E, 0x73, 0x90, 0xE1, 0x11, 0xC2, 0x08, 0x60, 0x27, 0x00, 0x00}
#define RDTS_SERVER_TX_UUID  {0x02, 0x00, 0x2E, 0xC7, 0x8a, 0x0E, 0x73, 0x90, 0xE1, 0x11, 0xC2, 0x08, 0x60, 0x27, 0x00, 0x00}
#define RDTS_SERVER_RX_UUID  {0x01, 0x00, 0x2E, 0xC7, 0x8a, 0x0E, 0x73, 0x90, 0xE1, 0x11, 0xC2, 0x08, 0x60, 0x27, 0x00, 0x00}

/* Private constants ---------------------------------------------------------*/
const uint8_t rdtsc_uuid128[]= RDTS_SERVICE_UUID;
/// State machine used to retrieve RDTS characteristics information
const struct prf_char_def_128 rdtsc_char[RDTSC_CHAR_MAX] =
{
    [RDTSC_SRV_TX_DATA_CHAR]    = {RDTS_SERVER_TX_UUID, ATT_MANDATORY, ATT_CHAR_PROP_NTF},
    [RDTSC_SRV_RX_DATA_CHAR]    = {RDTS_SERVER_RX_UUID, ATT_MANDATORY, ATT_CHAR_PROP_WR_NO_RESP},
};
/// State machine used to retrieve RDTS service characteristic description information
const struct prf_char_desc_def rdtsc_desc[RDTSC_DESC_MAX] =
{
    ///RDTS Client Config
    [RDTSC_SRV_TX_DATA_CLI_CFG]  = {ATT_DESC_CLIENT_CHAR_CFG, ATT_MANDATORY, RDTSC_SRV_TX_DATA_CHAR},
};

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void app_rdtsc_init(void)
{
    //register application subtask to app task
    struct prf_task_t prf;
    prf.prf_task_id = TASK_ID_RDTSC;
    prf.prf_task_handler = &app_rdtsc_handlers;
    ns_ble_prf_task_register(&prf);
    
    //register get itf function to prf.c
    struct prf_get_func_t get_func;
    get_func.task_id = TASK_ID_RDTSC;
    get_func.prf_itf_get_func = rdtsc_prf_itf_get;
    prf_get_itf_func_register(&get_func);
}

/** 
 * @brief Create raw data transfer server profile database.
 * @return void
 */
void app_rdtsc_add_rdts(void)
{
    NS_LOG_DEBUG("%s\r\n", __func__);
    struct rdtsc_db_cfg *db_cfg;

    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
                                                             TASK_GAPM,
                                                             TASK_APP,
                                                             gapm_profile_task_add_cmd,
                                                             sizeof(struct rdtsc_db_cfg));

    // Fill message
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = PERM(SVC_AUTH, NO_AUTH);
    req->prf_task_id = TASK_ID_RDTSC;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    // Set parameters
    db_cfg = (struct rdtsc_db_cfg *) req->param;
    
    // Attribute table. In case the handle offset needs to be saved
    db_cfg->p_char      = &rdtsc_char[0];
    db_cfg->p_desc      = &rdtsc_desc[0];
    db_cfg->p_uuid128   = rdtsc_uuid128;

    // Send the message
    ke_msg_send(req);
    
    app_rdtsc_init();
}


void app_rdtsc_enable_prf(uint8_t conidx)
{
    NS_LOG_DEBUG("%s\r\n", __func__);
    struct rdtsc_enable_req * req = KE_MSG_ALLOC(RDTSC_ENABLE_REQ,
                                                  prf_get_task_from_id(TASK_ID_RDTSC),
                                                  TASK_APP,
                                                  rdtsc_enable_req);
                                                        
    // Fill in the parameter structure
    req->conidx = conidx;
    req->con_type = PRF_CON_DISCOVERY; 
    
    // Send the message
    ke_msg_send(req);
}


void user_send_ble_data(const uint8_t *data, uint16_t length)
{
    NS_LOG_DEBUG("%s\r\n", __func__);
    struct rdtsc_data_tx_req * req = KE_MSG_ALLOC_DYN(RDTSC_DATA_TX_REQ,
                                                      prf_get_task_from_id(TASK_ID_RDTSC),
                                                      TASK_APP,
                                                      rdtsc_data_tx_req,
                                                      length);
    
    req->length = length;
    memcpy(&req->data[0], data, length);
    ke_msg_send(req);
}


/**
 ****************************************************************************************
 * @brief Handles client enable confirmation
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int rdtsc_enable_cfm_handler(ke_msg_id_t const msgid,
                             struct rdtsc_enable_cfm const *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    NS_LOG_DEBUG("%s\r\n", __func__);
    NS_LOG_INFO("RDTSC profile was enabled\r\n");
    
    ns_ble_get_peer_info(GAPC_GET_PEER_FEATURES);
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles confirmation that data has been sent
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int rdtsc_data_tx_cfm_handler(ke_msg_id_t const msgid,
                                      struct rdtsc_data_tx_cfm const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    NS_LOG_DEBUG("%s\r\n", __func__);
    usart_forward_to_ble_loop();
    
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles confirmation that data has been sent
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int rdtsc_data_rx_ind_handler(ke_msg_id_t const msgid,
                              struct rdtsc_data_rx_ind const *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    NS_LOG_DEBUG("%s\r\n", __func__);
    app_usart_tx_fifo_enter(param->data,param->length);
    
    return (KE_MSG_CONSUMED);
}

/// Default State handlers definition
const struct ke_msg_handler app_rdtsc_msg_handler_list[] =
{
    {RDTSC_ENABLE_CFM,                  (ke_msg_func_t)rdtsc_enable_cfm_handler},
    {RDTSC_DATA_TX_CFM,                 (ke_msg_func_t)rdtsc_data_tx_cfm_handler},
    {RDTSC_DATA_RX_IND,                 (ke_msg_func_t)rdtsc_data_rx_ind_handler},
};

const struct app_subtask_handlers app_rdtsc_handlers = APP_HANDLERS(app_rdtsc);


#endif //BLE_RDTS_CLIENT
