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
 * @file generic_onoff_client.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef GENERIC_ONOFF_CLIENT_H__
#define GENERIC_ONOFF_CLIENT_H__


#include <stdint.h>
#include "access.h"
#include "generic_onoff_common.h"
#include "generic_onoff_messages.h"
#include "access_reliable.h"

#define GENERIC_ONOFF_CLIENT_MODEL_ID 0x1001

typedef struct __generic_onoff_client_t generic_onoff_client_t;

typedef void (*generic_onoff_state_status_cb_t)(const generic_onoff_client_t * p_self,
                                                const access_message_rx_meta_t * p_meta,
                                                const generic_onoff_status_params_t * p_in);
typedef struct
{
    generic_onoff_state_status_cb_t onoff_status_cb;
    access_publish_timeout_cb_t periodic_publish_cb;
    access_reliable_cb_t ack_transaction_status_cb;
} generic_onoff_client_callbacks_t;

typedef struct
{
    uint32_t timeout;
    bool force_segmented;
    mesh_transmic_size_t transmic_size;
    const generic_onoff_client_callbacks_t * p_callbacks;
} generic_onoff_client_settings_t;

typedef union
{
    generic_onoff_set_msg_pkt_t set;
} generic_onoff_client_msg_data_t;

struct __generic_onoff_client_t
{
    access_model_handle_t model_handle;
    generic_onoff_client_msg_data_t msg_pkt;
    access_reliable_t access_message;
    generic_onoff_client_settings_t settings;
};

/**
 * @brief  Generic onoff client initialization.
 * @param  p_client Pointer to generic_onoff_client_t context.
 * @param  element_index specified element.
 * @return error code.
 */
uint32_t generic_onoff_client_init(generic_onoff_client_t * p_client, uint8_t element_index);

/**
 * @brief  Send out the set message with unack.
 * @param  p_client Pointer to generic_onoff_client_t context.
 * @param  p_params parameters related to the sent message.
 * @param  p_transition parameters related to the transition.
 * @param  repeats number of publishment.
 * @return error code.
 */
uint32_t generic_onoff_client_set_unack(generic_onoff_client_t * p_client, const generic_onoff_set_params_t * p_params,
                                        const model_transition_t * p_transition, uint8_t repeats);

/**
 * @brief  Reliable publishment.
 * @param  p_client Pointer to generic_onoff_client_t context.
 * @param  p_params parameters related to reliable publishment message.
 * @param  p_transition parameters related to transition.
 */
uint32_t generic_onoff_client_set(generic_onoff_client_t * p_client, const generic_onoff_set_params_t * p_params,
                                  const model_transition_t * p_transition);



#endif /* GENERIC_ONOFF_CLIENT_H__ */
