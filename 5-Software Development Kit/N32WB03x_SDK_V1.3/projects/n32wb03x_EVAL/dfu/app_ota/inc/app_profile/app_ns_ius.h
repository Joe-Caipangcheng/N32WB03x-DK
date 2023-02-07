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
 * @file app_ns_ius.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */


#ifndef __APP_NS_IUS_H__
#define __APP_NS_IUS_H__


/* Includes ------------------------------------------------------------------*/
#include "rwip_config.h"     // SW configuration
#include "prf_types.h"
#include "prf.h"

#if (BLE_APP_NS_IUS)


#define SERVICE_NS_IUS       {0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,  0x01,0x00,  0x11,0x11}        
#define CHAR_NS_IUS_RC       {0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11  ,0x02,0x00,  0x11,0x11}     
#define CHAR_NS_IUS_CC       {0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11  ,0x03,0x00,  0x11,0x11}        
/* Public typedef -----------------------------------------------------------*/
enum
{
    NS_IUS_IDX_SVC,
    NS_IUS_IDX_RC_CHAR,
    NS_IUS_IDX_RC_VAL,
    NS_IUS_IDX_RC_CFG,
    NS_IUS_IDX_CC_CHAR,
    NS_IUS_IDX_CC_VAL,
    NS_IUS_IDX_CC_CFG,
    NS_IUS_IDX_NB,
};

/* Public variables ---------------------------------------------------------*/
extern struct attm_desc_128 ns_ius_att_db[NS_IUS_IDX_NB];
extern const struct app_subtask_handlers ns_ius_app_handlers;


/* Public function prototypes -----------------------------------------------*/
/**
 * @brief SEND DATA THROUGH NS IUS SERVICE
 **/
void ns_ble_ius_app_cc_send(uint8_t *p_data, uint16_t length);
/**
 * @brief Add a NATIONS IMAGE UPDATE Service instance in the DB
 **/
void app_ns_ius_add_ns_ius(void);
/**
 * @brief Initialize NATIONS IMAGE UPDATE Service Application
 **/
void app_ns_ius_init(void);

#endif //BLE_APP_NS_IUS






#endif //__APP_NS_IUS_H__
