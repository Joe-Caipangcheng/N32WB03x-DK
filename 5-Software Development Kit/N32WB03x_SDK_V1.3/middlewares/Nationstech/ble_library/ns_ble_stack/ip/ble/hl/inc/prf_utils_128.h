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
 * @file prf_utils_128.h
 * @author Nations Firmware Team
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef _PRF_UTILS_128_H_
#define _PRF_UTILS_128_H_

/**
 ****************************************************************************************
 * @addtogroup PRF_UTILS
 * @ingroup PROFILE
 *
 * @brief Definitions of shared profiles functions that can be used by several profiles
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwprf_config.h"

#if (BLE_CLIENT_PRF)
#include "ke_msg.h"
#include "prf_types.h"
#include "gattc_task.h"
#include "gapc.h"
#include "gapc_task.h"
#include "attm_db.h"
#include "prf.h"
#endif /* (BLE_CLIENT_PRF) */

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

#if (BLE_CLIENT_PRF)

/// Characteristic definition
struct prf_char_def_128
{
    /// Characteristic UUID
    uint8_t uuid[ATT_UUID_128_LEN];
    /// Requirement Attribute Flag
    uint8_t req_flag;
    /// Mandatory Properties
    uint8_t prop_mand;
};

/**
 ****************************************************************************************
 * @brief Request service discovery with 128-bit UUID on peer device.
 *
 * This request will be used to retrieve start and end handles of the service.
 *
 * @param con_info Pointer to connection information (connection handle, app task id,
 *                 profile task id)
 *
 * @param uuid_128 128-bit service UUID
 ****************************************************************************************
 */
void prf_disc_svc_send_128(prf_env_t* env, uint8_t *uuid_128, uint8_t conidx);
 
/**
 ****************************************************************************************
 * @brief Check validity for service characteristic with 128-bit UUID
 *
 * For each characteristic in service it verifies handles.
 *
 * If some handles are not present, it checks if they shall be present or they are optional.
 *
 * @param nb_chars   Number of Characteristics in the service
 * @param chars      Characteristics values (char handles, val handles, properties)
 * @param chars_req  Characteristics requirements.
 *
 * @return 0x1 if service is valid, 0x00 else.
 ****************************************************************************************
 */
uint8_t prf_check_svc_char_validity_128(uint8_t nb_chars,
                                    const struct prf_char_inf* chars,
                                    const struct prf_char_def_128* chars_req);

/**
 ****************************************************************************************
 * @brief Look for requested characteristics in the list provided by the
 * gattc_disc_char_ind message.
 *
 * @param svc_ehdl         End handle of the service for which characteristics are discovered
 * @param nb_chars         Length of provided arrays (chars and chars_req)
 * @param chars            Characteristics
 * @param chars_req        Characteristics requirements
 * @param param            List of found characteristics
 * @param last_found_char  Characteristics code of the last characteristic that has been found.
 ****************************************************************************************
 */
void prf_search_chars_128(uint16_t svc_ehdl, uint8_t nb_chars,
                      struct prf_char_inf* chars, const struct prf_char_def_128* chars_req,
                      const struct gattc_disc_char_ind* param,
                      uint8_t* last_found_char);

void prf_disc_svc_sdp_send_128(prf_env_t *prf_env, uint8_t conidx,const uint8_t* uuid_128);
void prf_disc_char_by_uuid_128_send(prf_env_t *prf_env, uint8_t conidx, const uint8_t *uuid_128);
void prf_extract_svc_sdp_info_128(const struct gattc_sdp_svc_ind *param,
                                  uint8_t nb_chars,
                                  const struct prf_char_def_128* chars_req,
                                  struct prf_char_inf* chars,
                                  uint8_t nb_descs,
                                  const struct prf_char_desc_def* descs_req,
                                  struct prf_char_desc_inf* descs);

#endif //(BLE_CLIENT_PRF)


/// @} prf_utils

#endif /* _PRF_UTILS_128_H_ */
