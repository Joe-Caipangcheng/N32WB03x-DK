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
 * @file rdts_common.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef __RDTS_COMMON_H__
#define __RDTS_COMMON_H__

#include "gattc_task.h"

/** 
 * @brief Validate the value of the Client Characteristic CFG.
 * @param[in] is_notification indicates whether the CFG is Notification (true) or Indication (false)
 * @param[in] param Pointer to the parameters of the message.
 * @return status. 
 */
int check_client_char_cfg(bool is_notification, const struct gattc_write_req_ind *param);

/** 
 * @brief Find the handle of the Characteristic Value having as input the Client Characteristic CFG handle
 * @param[in] cfg_handle the Client Characteristic CFG handle
 * @return the corresponding value handle 
 */
uint16_t get_value_handle(uint16_t cfg_handle);

/** 
 * @brief Find the handle of Client Characteristic CFG having as input the Characteristic value handle
 * @param[in] value_handle the Characteristic value handle
 * @return the corresponding Client Characteristic CFG handle 
 */
uint16_t get_cfg_handle(uint16_t value_handle);

#if (BLE_RDTSS_SERVER)
/** 
 * @brief Compute the handle of a given attribute based on its index
 * @details Specific to raw data transfer server in 128bit uuid
 * @param[in] att_idx attribute index
 * @return the corresponding handle 
 */
uint16_t rdtss_get_att_handle(uint8_t att_idx);

/** 
 * @brief Compute the handle of a given attribute based on its index
 * @details Specific to raw data transfer server in 128bit uuid
 * @param[in] handle attribute handle
 * @param[out] att_idx attribute index
 * @return high layer error code 
 */
uint8_t rdtss_get_att_idx(uint16_t handle, uint8_t *att_idx);
#endif // (BLE_RDTSS_SERVER)


#if (BLE_RDTSS_16BIT_SERVER)
/** 
 * @brief Compute the handle of a given attribute based on its index
 * @details Specific to raw data transfer server in 16bit uuid
 * @param[in] att_idx attribute index
 * @return the corresponding handle 
 */
uint16_t rdtss_16bit_get_att_handle(uint8_t att_idx);

/** 
 * @brief Compute the handle of a given attribute based on its index
 * @details Specific to raw data transfer server in 16bit uuid
 * @param[in] handle attribute handle
 * @param[out] att_idx attribute index
 * @return high layer error code 
 */
uint8_t rdtss_16bit_get_att_idx(uint16_t handle, uint8_t *att_idx);
#endif // (BLE_RDTSS_16BIT_SERVER)


#endif // __RDTS_COMMON_H__
