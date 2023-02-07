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
 * @file ns_sec.h
 * @author Nations Firmware Team
 * @version v1.0.3
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** @addtogroup NS_SEC
 * @{
 */

#ifndef NS_SEC_H_
#define NS_SEC_H_

/* Includes ------------------------------------------------------------------*/
#include "rwip_config.h"

#if (BLE_APP_SEC)
#include "global_func.h"
/* Public define ------------------------------------------------------------*/
/* Public typedef -----------------------------------------------------------*/


enum sec_msg_id
{
    NS_SEC_NULL_MSG = 0,
    NS_SEC_BOND_STATE,
    NS_SEC_PAIRING_RSP,
    NS_SEC_PINCODE_DISPLAY,
    NS_SEC_PINCODE_ENTER,
    NS_SEC_PAIR_SUCCEED,
    NS_SEC_PAIR_FAILED,
    NS_SEC_LTK_FOUND,
    NS_SEC_LTK_MISSING,
    NS_SEC_ENC_SUCCEED,
    
};

struct sec_msg_t
{
    enum sec_msg_id msg_id;
    //income message
    union 
    {
        uint8_t const*  peer_num;
        uint32_t const* pincode;
        struct gapc_bond_cfm* cfm;
    }msg;
    
};


/* bonding structure
 * --------------------------------------------------------------------------------
 * |         ADDR            |                 FIELD                   |   SIZE   |
 * --------------------------|-----------------------------------------|----------|
 * | BOND_DATA_BASE_ADDR+0   | Bond space valid flag                   |    2     |
 * --------------------------|-----------------------------------------|----------|
 * | BOND_DATA_BASE_ADDR+2   | Bonded peer device numbers              |    1     |
 * --------------------------------------------------------------------------------
 * | BOND_DATA_BASE_ADDR+4   | Bond data0(@ app_sec_bond_data_env_tag) |          |
 * --------------------------------------------------------------------------------
 * |                         | Bond data1(@ app_sec_bond_data_env_tag) |          |
 * --------------------------------------------------------------------------------
 * |                         |               ......                    |          |
 * --------------------------------------------------------------------------------
 */  

/// Application Security Bond Data environment structure
struct app_sec_bond_data_env_tag
{
    // LTK
    struct gap_sec_key ltk;
    
    // Random Number
    rand_nb_t rand_nb;
    
    // EDIV
    uint16_t ediv;
    
    // Remote IRK
    struct gapc_irk irk;

    // LTK key size
    uint8_t key_size;

    // Last paired peer address type
    uint8_t peer_addr_type;
    
    // Last paired peer address
    bd_addr_t peer_addr;

    // authentication level
    uint8_t auth;
};

struct local_device_bond_data_tag
{
    uint16_t valid_flag;
    uint8_t num;
    struct app_sec_bond_data_env_tag single_peer_bond_data[8]; //MAX_BOND_PEER
};



struct ns_sec_init_t
{
    //Pairing Features 
    struct gapc_pairing pairing_feat;
    
    //pin code for pairing
    uint32_t pin_code;
    bool     rand_pin_enable;
    
    //config bond
    uint8_t  bond_enable;
    uint8_t  bond_max_peer;
    uint16_t bond_sync_delay;
    uint32_t bond_db_addr;
    
    //sec msg hanlder
    void (*ns_sec_msg_handler)(struct sec_msg_t const*);    
    
};

struct app_sec_env_tag
{
    // Bond status
    bool bonded;
    //Pairing Features 
    struct ns_sec_init_t sec_init;
};

/* Public define ------------------------------------------------------------*/ 
/* Public constants ---------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
extern const struct app_subtask_handlers app_sec_handlers;
/* Public function prototypes -----------------------------------------------*/

/**
 * @brief  Initialize the Application Security Module
 */
void ns_sec_init(struct ns_sec_init_t const* init);

/**
 * @brief Send a security request to the peer device. This function is used to require the
 * central to start the encryption with a LTK that would have shared during a previous
 * bond procedure.
 *
 * @param[in]   - conidx: Connection Index
 */
void ns_sec_send_security_req(uint8_t conidx);

/**
 * @brief  get the bond status
 */
bool ns_sec_get_bond_status(void);
/**
 * @brief  erase all bond information
 */
void ns_sec_bond_db_erase_all(void);

/**
 * @brief  store bond information event hander
 */
int  ns_sec_bond_store_evt_handler(ke_msg_id_t const msgid, void *p_param, \
                                  ke_task_id_t const dest_id, ke_task_id_t const src_id);

uint8_t ns_sec_get_iocap(void);

void ns_bond_last_bonded_addr(struct gap_bdaddr *p_addr);
void ns_bond_last_bonded_peer_id(struct gap_bdaddr *p_addr);
void ns_bond_last_bonded_ral_info(struct gap_ral_dev_info *ral_list);
#endif //(BLE_APP_SEC)

#endif // APP_SEC_H_

/// @} NS_SEC
