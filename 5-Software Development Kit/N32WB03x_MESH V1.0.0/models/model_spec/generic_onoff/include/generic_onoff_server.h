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
 * @file generic_onoff_server.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef GENERIC_ONOFF_SERVER_H__
#define GENERIC_ONOFF_SERVER_H__

#include <stdint.h>
#include "access.h"
#include "generic_onoff_common.h"
#include "model_common.h"

#define GENERIC_ONOFF_SERVER_MODEL_ID 0x1000

typedef struct __generic_onoff_server_t generic_onoff_server_t;
typedef void (*generic_onoff_state_set_cb_t)(const generic_onoff_server_t * p_self,
                                             const access_message_rx_meta_t * p_meta,
                                             const generic_onoff_set_params_t * p_in,
                                             const model_transition_t * p_in_transition,
                                             generic_onoff_status_params_t * p_out);
typedef void (*generic_onoff_state_get_cb_t)(const generic_onoff_server_t * p_self,
                                             const access_message_rx_meta_t * p_meta,
                                             generic_onoff_status_params_t * p_out);
typedef struct
{
    generic_onoff_state_set_cb_t    set_cb;
    generic_onoff_state_get_cb_t    get_cb;
} generic_onoff_server_state_cbs_t;
typedef struct
{
    generic_onoff_server_state_cbs_t onoff_cbs;
} generic_onoff_server_callbacks_t;
typedef struct
{
    bool force_segmented;
    mesh_transmic_size_t transmic_size;
    const generic_onoff_server_callbacks_t * p_callbacks;
} generic_onoff_server_settings_t;

struct __generic_onoff_server_t
{
    access_model_handle_t model_handle;
    tid_tracker_t tid_tracker;
    generic_onoff_server_settings_t settings;
};
/**
 * @brief  Generic onoff server initialization.
 * @param  p_server Pointer to generic_onoff_server_t context.
 * @param  element_index specified element.
 * @return error code.
 */
uint32_t generic_onoff_server_init(generic_onoff_server_t * p_server, uint8_t element_index);

/**
 * @brief  p_server Pointer to generic_onoff_server_t context.
 * @param  p_params parameters related to the onoff status.
 * @return error code.
 */
uint32_t generic_onoff_server_status_publish(generic_onoff_server_t * p_server, const generic_onoff_status_params_t * p_params);

#endif /* GENERIC_ONOFF_SERVER_H__ */
