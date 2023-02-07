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
 * @file rdts_common.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

 /* Includes ------------------------------------------------------------------*/
#include "rwip_config.h"              // SW configuration

#if (BLE_RDTS_ENABLE)
#include "rdts_common.h"
#if (BLE_RDTSS_SERVER)
#include "rdtss.h"
#endif // (BLE_RDTSS_SERVER)

#if (BLE_RDTSS_16BIT_SERVER)
#include "rdtss_16bit.h"
#endif // (BLE_RDTSS_16BIT_SERVER)

#include "gattc_task.h"
#include "att.h"
#include "attm_db.h"
#include "prf_types.h"
#include "prf_utils.h"
/* Private define ------------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

int check_client_char_cfg(bool is_notification, const struct gattc_write_req_ind *param)
{
    uint8_t status = GAP_ERR_NO_ERROR;
    uint16_t ntf_cfg = 0;

    if (param->length != sizeof(uint16_t))
    {
        status =  ATT_ERR_INVALID_ATTRIBUTE_VAL_LEN;
    }
    else
    {
        ntf_cfg = *((uint16_t*)param->value);

        if (is_notification)
        {
            if ( (ntf_cfg != PRF_CLI_STOP_NTFIND) && (ntf_cfg != PRF_CLI_START_NTF) )
            {
                status =  PRF_ERR_INVALID_PARAM;
            }
        }
        else
        {
            if ( (ntf_cfg != PRF_CLI_STOP_NTFIND) && (ntf_cfg != PRF_CLI_START_IND) )
            {
                status =  PRF_ERR_INVALID_PARAM;
            }
        }
    }

    return status;
}

uint16_t get_value_handle(uint16_t cfg_handle)
{
    uint8_t uuid[ATT_UUID_128_LEN];
    uint8_t uuid_len;
    uint16_t handle = cfg_handle;
    struct attm_svc *srv;

    srv = attmdb_get_service(handle);

    while ((handle >= srv->svc.start_hdl) && (handle <= srv->svc.end_hdl))
    {
        struct attm_elmt elmt;

        // Retrieve UUID
        attmdb_get_attribute(handle, &elmt);
        attmdb_get_uuid(&elmt, &uuid_len, uuid, false, false);

        // check for Characteristic declaration
        if (*(uint16_t *)&uuid[0] == ATT_DECL_CHARACTERISTIC)
            return handle + 1;

        handle--;
    }

    return 0;  //Should not reach this point. something is wrong with the database
}

uint16_t get_cfg_handle(uint16_t value_handle)
{
    uint8_t uuid[ATT_UUID_128_LEN];
    uint8_t uuid_len;
    uint16_t handle = value_handle;
    struct attm_svc *srv;

    srv = attmdb_get_service(handle);

    /* Iterate the database to find the client characteristic configuration.
    */
    while ((handle >= srv->svc.start_hdl) && (handle <= srv->svc.end_hdl))
    {
        struct attm_elmt elmt;

        // Retrieve UUID
        attmdb_get_attribute(handle, &elmt);
        attmdb_get_uuid(&elmt, &uuid_len, uuid, false, false);

        // check for Client Characteristic Configuration
        if (*(uint16_t *)&uuid[0] == ATT_DESC_CLIENT_CHAR_CFG && uuid_len == sizeof(uint16_t))
            return handle;
        else if (*(uint16_t *)&uuid[0] == ATT_DECL_CHARACTERISTIC && uuid_len == sizeof(uint16_t))
            break; // found the next Characteristic declaration without findig a CC CFG,

        handle++;
    }

    return 0;  //Should not reach this point. something is wrong with the database
}

#if (BLE_RDTSS_SERVER)
uint16_t rdtss_get_att_handle(uint8_t att_idx)
{
    struct rdtss_env_tag *rdtss_env = PRF_ENV_GET(RDTSS, rdtss);
    uint16_t handle = ATT_INVALID_HDL;

    if (att_idx < rdtss_env->max_nb_att)
    {
        handle = rdtss_env->shdl + att_idx;
    }

    return handle;
}

uint8_t rdtss_get_att_idx(uint16_t handle, uint8_t *att_idx)
{
    struct rdtss_env_tag *rdtss_env = PRF_ENV_GET(RDTSS, rdtss);
    uint8_t status = PRF_APP_ERROR;

    if ((handle >= rdtss_env->shdl) && (handle < rdtss_env->shdl + rdtss_env->max_nb_att))
    {
        *att_idx = handle - rdtss_env->shdl;
        status = ATT_ERR_NO_ERROR;
    }

    return status;
}
#endif // (BLE_RDTSS_SERVER)


#if (BLE_RDTSS_16BIT_SERVER)
uint16_t rdtss_16bit_get_att_handle(uint8_t att_idx)
{
    struct rdtss_16bit_env_tag *rdtss_16bit_env = PRF_ENV_GET(RDTSS_16BIT, rdtss_16bit);
    uint16_t handle = ATT_INVALID_HDL;

    if (att_idx < rdtss_16bit_env->max_nb_att)
    {
        handle = rdtss_16bit_env->shdl + att_idx;
    }

    return handle;
}

uint8_t rdtss_16bit_get_att_idx(uint16_t handle, uint8_t *att_idx)
{
    struct rdtss_16bit_env_tag *rdtss_16bit_env = PRF_ENV_GET(RDTSS_16BIT, rdtss_16bit);
    uint8_t status = PRF_APP_ERROR;

    if ((handle >= rdtss_16bit_env->shdl) && (handle < rdtss_16bit_env->shdl + rdtss_16bit_env->max_nb_att))
    {
        *att_idx = handle - rdtss_16bit_env->shdl;
        status = ATT_ERR_NO_ERROR;
    }

    return status;
}
#endif // (BLE_RDTSS_16BIT_SERVER)


#endif // (BLE_RDTS_ENABLE)
