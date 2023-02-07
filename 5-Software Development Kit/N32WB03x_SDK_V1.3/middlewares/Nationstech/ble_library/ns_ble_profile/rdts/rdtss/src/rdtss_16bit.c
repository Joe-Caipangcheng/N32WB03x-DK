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
 * @file rdtss_16bit.c
 * @author Nations Firmware Team
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

 /* Includes ------------------------------------------------------------------*/
#include "rwip_config.h"              // SW configuration

#if (BLE_RDTSS_16BIT_SERVER)
#include "rdtss_16bit.h"
#include "rdtss_16bit_task.h"
#include "attm_db.h"
#include "gapc.h"
#include "prf.h"
#include "ke_mem.h"


/* Private functions ---------------------------------------------------------*/

/** 
 * @brief Initialization of the RDTSS 16bit uuid module.
 * This function performs all the initializations of the Profile module.
 *  - Creation of database (if it's a service)
 *  - Allocation of profile required memory
 *  - Initialization of task descriptor to register application
 *      - Task State array
 *      - Number of tasks
 *      - Default task handler
 *
 * @param[out]    env        Collector or Service allocated environment data.
 * @param[in|out] start_hdl  Service start handle (0 - dynamically allocated), only applies for services.
 * @param[in]     app_task   Application task number.
 * @param[in]     sec_lvl    Security level (AUTH, EKS and MI field of @see enum attm_value_perm_mask)
 * @param[in]     param      Configuration parameters of profile collector or service (32 bits aligned)
 *
 * @return status code to know if profile initialization succeed or not. 
 */
static uint8_t rdtss_16bit_init(struct prf_task_env *env, uint16_t *start_hdl, uint16_t app_task, uint8_t sec_lvl, struct rdtss_16bit_db_cfg *params)
{
    //------------------ create the attribute database for the profile -------------------

    // DB Creation Status
    uint8_t status = ATT_ERR_NO_ERROR;

    uint32_t cfg_flag = ((1<<params->max_nb_att)-1);
    
    status = attm_svc_create_db(start_hdl, *(params->svc_uuid), (uint8_t*)&cfg_flag,
            params->max_nb_att, NULL, env->task, params->att_tbl,
            (sec_lvl & PERM_MASK_SVC_AUTH) | (sec_lvl & PERM_MASK_SVC_EKS) | PERM(SVC_SECONDARY, DISABLE) );    

    //-------------------- allocate memory required for the profile  ---------------------
    if (status == ATT_ERR_NO_ERROR)
    {
        struct rdtss_16bit_env_tag *rdtss_16bit_env =
                (struct rdtss_16bit_env_tag *) ke_malloc(sizeof(struct rdtss_16bit_env_tag), KE_MEM_ATT_DB);

        //load config from init 
        memcpy(&(rdtss_16bit_env->db_cfg),params,sizeof(struct rdtss_16bit_db_cfg));        
        // allocate RDTSS_16BITrequired environment variable
        env->env = (prf_env_t *)rdtss_16bit_env;
        rdtss_16bit_env->shdl = *start_hdl;
        rdtss_16bit_env->max_nb_att = params->max_nb_att;
        rdtss_16bit_env->prf_env.app_task = app_task
                | (PERM_GET(sec_lvl, SVC_MI) ? PERM(PRF_MI, ENABLE) : PERM(PRF_MI, DISABLE));
        rdtss_16bit_env->prf_env.prf_task = env->task | PERM(PRF_MI, DISABLE);

        // initialize environment variable
        env->id                     = TASK_ID_RDTSS_16BIT;

        rdtss_16bit_task_init(&(env->desc));
        
        co_list_init(&(rdtss_16bit_env->values));
        rdtss_16bit_init_ccc_values(params->att_tbl, params->max_nb_att);

        // service is ready, go into an Idle state
        ke_state_set(env->task, RDTSS_16BIT_IDLE);
    }

    return status;
}
/** 
 * @brief Destruction of the RDTSS_16BITmodule - due to a reset for instance.
 * This function clean-up allocated memory (attribute database is destroyed by another
 * procedure)
 *
 * @param[in|out]    env        Collector or Service allocated environment data. 
 */
static void rdtss_16bit_destroy(struct prf_task_env *env)
{
    struct rdtss_16bit_env_tag *rdtss_16bit_env = (struct rdtss_16bit_env_tag *)env->env;

    // remove all values present in list
    while (!co_list_is_empty(&(rdtss_16bit_env->values)))
    {
        struct co_list_hdr *hdr = co_list_pop_front(&(rdtss_16bit_env->values));
        ke_free(hdr);
    }

    // free profile environment variables
    env->env = NULL;
    ke_free(rdtss_16bit_env);
}

/** 
 * @brief Handles Connection creation
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 * @param[in]        conidx     Connection index 
 */
static void rdtss_16bit_create(struct prf_task_env *env, uint8_t conidx)
{
    int att_idx;
    struct rdtss_16bit_env_tag *rdtss_16bit_env = (struct rdtss_16bit_env_tag *)env->env;
    ASSERT_ERR(conidx < BLE_CONNECTION_MAX);

    // Find all ccc fields and clean them
    for (att_idx = 1; att_idx < rdtss_16bit_env->max_nb_att; att_idx++)
    {
        // Find only CCC characteristics
        if( PERM_GET(rdtss_16bit_env->db_cfg.att_tbl[att_idx].perm, UUID_LEN) == 0 &&
            (rdtss_16bit_env->db_cfg.att_tbl[att_idx].uuid) == ATT_DESC_CLIENT_CHAR_CFG
        )
        {
            // Clear CCC value
            rdtss_16bit_set_ccc_value(conidx, att_idx, 0);
        }
    }
}

/** 
 * @brief Handles Disconnection
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 * @param[in]        conidx     Connection index
 * @param[in]        reason     Detach reason 
 */
static void rdtss_16bit_cleanup(struct prf_task_env *env, uint8_t conidx, uint8_t reason)
{
    int att_idx;
    
    struct rdtss_16bit_env_tag *rdtss_16bit_env = (struct rdtss_16bit_env_tag *)env->env;
    
    ASSERT_ERR(conidx < BLE_CONNECTION_MAX);

    // Find all ccc fields and clean them
    for (att_idx = 1; att_idx < rdtss_16bit_env->max_nb_att; att_idx++)
    {
        // Find only CCC characteristics
        if( PERM_GET(rdtss_16bit_env->db_cfg.att_tbl[att_idx].perm, UUID_LEN) == 0 &&
            (rdtss_16bit_env->db_cfg.att_tbl[att_idx].uuid) == ATT_DESC_CLIENT_CHAR_CFG
        )
        {
            // Clear CCC value
            rdtss_16bit_set_ccc_value(conidx, att_idx, 0);
        }
    }
    
    ke_state_set(prf_src_task_get(&(rdtss_16bit_env->prf_env), conidx), RDTSS_16BIT_IDLE);
}

/* Public variables ---------------------------------------------------------*/

/// RDTSS_16BITTask interface required by profile manager
const struct prf_task_cbs rdtss_16bit_itf =
{
        (prf_init_fnct) rdtss_16bit_init,
        rdtss_16bit_destroy,
        rdtss_16bit_create,
        rdtss_16bit_cleanup,
};

/* Private functions ---------------------------------------------------------*/

const struct prf_task_cbs* rdtss_16bit_prf_itf_get(void)
{
    return &rdtss_16bit_itf;
}

#endif // (BLE_RDTSS_16BIT_SERVER)
