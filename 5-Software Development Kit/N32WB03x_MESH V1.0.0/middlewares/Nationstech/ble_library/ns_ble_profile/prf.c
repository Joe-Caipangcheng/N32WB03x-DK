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
 * @file prf.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** @addtogroup 
 * @{
 */

#include "rwip_config.h"

#if (BLE_PROFILES)
#include "rwip.h"
#include "prf.h"
#include "att.h"
#include "mesh_gatt_srv.h"
struct prf_env_tag prf_env;



/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
static const struct prf_task_cbs * prf_itf_get(uint16_t task_id)
{
    const struct prf_task_cbs* prf_cbs = NULL;

    switch(KE_TYPE_GET(task_id))
    {
        case TASK_ID_MESH_GATT:{
                prf_cbs = mesh_gatt_prf_itf_get();
			}break;
        default:  break;
    }

    return prf_cbs;
}


/**
 ****************************************************************************************
 * @brief Perform Profile initialization
 *
 * @param[in] init_type  Type of initialization (@see enum rwip_init_type)
 ****************************************************************************************
 */
void prf_init(uint8_t init_type)
{
    uint8_t i;

    for(i = 0; i < BLE_NB_PROFILES ; i++)
    {
        switch (init_type)
        {
            case RWIP_INIT:
            {
                // FW boot profile initialization
                prf_env.prf[i].task = TASK_GAPC + i +1;
                ke_task_delete(prf_env.prf[i].task);    //TODO CHECK
                ke_task_create(prf_env.prf[i].task, &(prf_env.prf[i].desc));
            }
            break;

            case RWIP_RST:
            {
                // FW boot profile destruction
                // Get Profile API
                const struct prf_task_cbs * cbs = prf_itf_get(prf_env.prf[i].id);

                if(cbs != NULL)
                {
                    // request to destroy profile
                    cbs->destroy(&(prf_env.prf[i]));
                }
                // Request kernel to flush task messages
                ke_task_msg_flush(KE_TYPE_GET(prf_env.prf[i].task));
            }
            // No break

            case RWIP_1ST_RST:
            {
                // FW boot profile destruction
                prf_env.prf[i].env  = NULL;
                // unregister profile
                prf_env.prf[i].id   = TASK_ID_INVALID;
                prf_env.prf[i].desc.msg_handler_tab = NULL;
                prf_env.prf[i].desc.state           = NULL;
                prf_env.prf[i].desc.idx_max         = 0;
                prf_env.prf[i].desc.msg_cnt         = 0;
            }
            break;

            default:
            {
                // Do nothing
            }
            break;
        }
    }
}

/**
 ****************************************************************************************
 * @brief Create Profile (collector or service) task creation and initialize it.
 *
 * @param[in|out] params   Collector or Service parameter used for profile task creation
 * @param[out]    prf_task Allocated Task number
 *
 * @return status of adding profile task
 ****************************************************************************************
 */
uint8_t prf_add_profile(struct gapm_profile_task_add_cmd * params, ke_task_id_t* prf_task)
{
    uint8_t i;
    uint8_t status = GAP_ERR_NO_ERROR;

    // retrieve profile callback
    const struct prf_task_cbs * cbs = prf_itf_get(params->prf_task_id);
    if(cbs == NULL)
    {
        // profile API not available
        status = GAP_ERR_INVALID_PARAM;
    }

    // check if profile not already present in task list
    if(status == GAP_ERR_NO_ERROR)
    {
        for(i = 0; i < BLE_NB_PROFILES ; i++)
        {
            if(prf_env.prf[i].id == params->prf_task_id)
            {
                status = GAP_ERR_NOT_SUPPORTED;
                break;
            }
        }
    }

    if(status == GAP_ERR_NO_ERROR)
    {
        // find first available task
        for(i = 0; i < BLE_NB_PROFILES ; i++)
        {
            // available task found
            if(prf_env.prf[i].id == TASK_ID_INVALID)
            {
                // initialize profile
                status = cbs->init(&(prf_env.prf[i]), &(params->start_hdl), params->app_task, params->sec_lvl, params->param);

                // initialization succeed
                if(status == GAP_ERR_NO_ERROR)
                {
                    // register profile
                    prf_env.prf[i].id = params->prf_task_id;
                    *prf_task = prf_env.prf[i].task;
                }
                break;
            }
        }

        if(i == BLE_NB_PROFILES)
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
    }

    return (status);
}



/**
 ****************************************************************************************
 * @brief Link creation event, update profiles states.
 *
 * @param[in] conidx        connection index
 *
 ****************************************************************************************
 */
void prf_create(uint8_t conidx)
{
    uint8_t i;
    /* simple connection creation handler, nothing to do. */

    // execute create function of each profiles
    for(i = 0; i < BLE_NB_PROFILES ; i++)
    {
        // Get Profile API
        const struct prf_task_cbs * cbs = prf_itf_get(prf_env.prf[i].id);
        if(cbs != NULL)
        {
            // call create callback
            cbs->create(&(prf_env.prf[i]), conidx);
        }
    }
}

/**
 ****************************************************************************************
 * @brief Link disconnection event, clean-up profiles.
 *
 * @param[in] conidx        connection index
 * @param[in] reason        detach reason
 *
 ****************************************************************************************
 */
void prf_cleanup(uint8_t conidx, uint8_t reason)
{
    uint8_t i;
    /* simple connection creation handler, nothing to do. */

    // execute create function of each profiles
    for(i = 0; i < BLE_NB_PROFILES ; i++)
    {
        // Get Profile API
        const struct prf_task_cbs * cbs = prf_itf_get(prf_env.prf[i].id);
        if(cbs != NULL)
        {
            // call cleanup callback
            cbs->cleanup(&(prf_env.prf[i]), conidx, reason);
        }
    }
}

/**
 ****************************************************************************************
 * @brief Retrieve environment variable allocated for a profile
 *
 * @param[in] prf_id        Profile Task Identifier
 *
 * @return Environment variable allocated for a profile
 ****************************************************************************************
 */
prf_env_t* prf_env_get(uint16_t prf_id)
{
    prf_env_t* env = NULL;
    uint8_t i;
    // find if profile present in profile tasks
    for(i = 0; i < BLE_NB_PROFILES ; i++)
    {
        // check if profile identifier is known
        if(prf_env.prf[i].id == prf_id)
        {
            env = prf_env.prf[i].env;
            break;
        }
    }

    return env;
}
/**
 ****************************************************************************************
 * @brief Retrieve source profile task number value
 *
 * @param[in] env     Profile Environment
 * @param[in] conidx  Connection index
 *
 * @return Source profile task number value
 ****************************************************************************************
 */
ke_task_id_t prf_src_task_get(prf_env_t* env, uint8_t conidx)
{
    ke_task_id_t task = PERM_GET(env->prf_task, PRF_TASK);

    if(PERM_GET(env->prf_task, PRF_MI))
    {
        task = KE_BUILD_ID(task, conidx);
    }

    return task;
}
/**
 ****************************************************************************************
 * @brief Retrieve destination application task number value
 *
 * @param[in] env     Profile Environment
 * @param[in] conidx  Connection index
 *
 * @return Destination application task number value
 ****************************************************************************************
 */
ke_task_id_t prf_dst_task_get(prf_env_t* env, uint8_t conidx)
{
    ke_task_id_t task = PERM_GET(env->app_task, PRF_TASK);

    if(PERM_GET(env->app_task, PRF_MI))
    {
        task = KE_BUILD_ID(task, conidx);
    }

    return task;
}

/**
 ****************************************************************************************
 * @brief Retrieve Task Identifier from Task number
 * (automatically update index of task in returned task id)
 *
 * @param task Task number
 * @return Task Identifier
 ****************************************************************************************
 */
ke_task_id_t prf_get_id_from_task(ke_msg_id_t task)
{
    ke_task_id_t id = TASK_ID_INVALID;
    uint8_t idx = KE_IDX_GET(task);
    uint8_t i;
    task = KE_TYPE_GET(task);

    // find if profile present in profile tasks
    for(i = 0; i < BLE_NB_PROFILES ; i++)
    {
        // check if profile identifier is known
        if(prf_env.prf[i].task == task)
        {
            id = prf_env.prf[i].id;
            break;
        }
    }

    return KE_BUILD_ID(id, idx);
}
/**
 ****************************************************************************************
 * @brief Retrieve Task Number from Task Identifier
 * (automatically update index of task in returned task id)
 *
 * @param id Task Identifier
 * @return Task Number
 ****************************************************************************************
 */
ke_task_id_t prf_get_task_from_id(ke_msg_id_t id)
{
    ke_task_id_t task = TASK_NONE;
    uint8_t idx = KE_IDX_GET(id);
    uint8_t i;
    id = KE_TYPE_GET(id);

    // find if profile present in profile tasks
    for(i = 0; i < BLE_NB_PROFILES ; i++)
    {
        // check if profile identifier is known
        if(prf_env.prf[i].id == id)
        {
            task = prf_env.prf[i].task;
            break;
        }
    }

    return KE_BUILD_ID(task, idx);
}


#endif // (BLE_PROFILES)

/// @} PRF
