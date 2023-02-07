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
 * @file generic_onoff_server.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
 
 
 
#include "generic_onoff_server.h"
#include "generic_onoff_common.h"
#include "generic_onoff_messages.h"
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "access.h"
#include "access_config.h"
#include "mesh_utils.h"
#include "mesh_common.h"
#include "mesh_error.h"
#include "core_tx.h"


/**
 * @brief  Send out the onoff state message.
 * @param  p_server point to the generic onoff server instance.
 * @param  p_message received message. If publish, it will be null.
 * @param  p_params paramters which will be sent out. 
 * @return error code.
 */
static uint32_t status_send(generic_onoff_server_t * p_server,
                            const access_message_rx_t * p_message,
                            const generic_onoff_status_params_t * p_params)
{
    generic_onoff_status_msg_pkt_t msg_pkt;

    if (p_params->present_on_off > GENERIC_ONOFF_MAX ||
        p_params->target_on_off  > GENERIC_ONOFF_MAX ||
        p_params->remaining_time_ms > TRANSITION_TIME_STEP_10M_MAX)
    {
        return MESH_ERROR_INVALID_PARAM;
    }

    msg_pkt.present_on_off = p_params->present_on_off;
    if (p_params->remaining_time_ms > 0)
    {
        msg_pkt.target_on_off = p_params->target_on_off;
        msg_pkt.remaining_time = model_transition_time_encode(p_params->remaining_time_ms);
    }

    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(GENERIC_ONOFF_OPCODE_STATUS),
        .p_buffer = (const uint8_t *) &msg_pkt,
        .length = p_params->remaining_time_ms > 0 ? GENERIC_ONOFF_STATUS_MAXLEN : GENERIC_ONOFF_STATUS_MINLEN,
        .force_segmented = p_server->settings.force_segmented,
        .transmic_size = p_server->settings.transmic_size
    };

    if (p_message == NULL)
    {
        return access_model_publish(p_server->model_handle, &reply);
    }
    else
    {
        return access_model_reply(p_server->model_handle, p_message, &reply);
    }
}
/**
 * @brief  Function for publish periodically.
 * @param  handle Model handle which it will be published on.
 * @param  p_args arguments pointer for the publish callback.
 */
static void periodic_publish_cb(access_model_handle_t handle, void * p_args)
{
    generic_onoff_server_t * p_server = (generic_onoff_server_t *)p_args;
    generic_onoff_status_params_t out_data = {0};

    p_server->settings.p_callbacks->onoff_cbs.get_cb(p_server, NULL, &out_data);
    (void) status_send(p_server, NULL, &out_data);
}
/**
 * @brief  Validate the input parameters.
 * @param  p_rx_msg received message.
 * @param  p_params parameters included in the received packet.
 * @return true if parameters are valid, otherwise false.
 */
static inline bool set_params_validate(const access_message_rx_t * p_rx_msg, const generic_onoff_set_msg_pkt_t * p_params)
{
    return (
            (p_rx_msg->length == GENERIC_ONOFF_SET_MINLEN || p_rx_msg->length == GENERIC_ONOFF_SET_MAXLEN) &&
            (p_params->on_off <= GENERIC_ONOFF_MAX)
           );
}
/**
 * @brief  Handle the onoff set message.
 * @param  model_handle model handle.
 * @param  p_rx_msg received message. 
 * @param  p_args arguments pointer for the handler.
 */
static void handle_set(access_model_handle_t model_handle, const access_message_rx_t * p_rx_msg, void * p_args)
{
//    printf("handle_set  \r\n");
    generic_onoff_server_t * p_server = (generic_onoff_server_t *) p_args;
    generic_onoff_set_params_t in_data = {0};
    model_transition_t in_data_tr = {0};
    generic_onoff_status_params_t out_data = {0};
    generic_onoff_set_msg_pkt_t * p_msg_params_packed = (generic_onoff_set_msg_pkt_t *) p_rx_msg->p_data;

    if (set_params_validate(p_rx_msg, p_msg_params_packed))
    {
        in_data.on_off = p_msg_params_packed->on_off;
        in_data.tid = p_msg_params_packed->tid;

        if (model_tid_validate(&p_server->tid_tracker, &p_rx_msg->meta_data, GENERIC_ONOFF_OPCODE_SET, in_data.tid))
        {
            if (p_rx_msg->length == GENERIC_ONOFF_SET_MAXLEN)
            {
                if (!model_transition_time_is_valid(p_msg_params_packed->transition_time))
                {
                    return;
                }

                in_data_tr.transition_time_ms = model_transition_time_decode(p_msg_params_packed->transition_time);
                in_data_tr.delay_ms = model_delay_decode(p_msg_params_packed->delay);
            }

            p_server->settings.p_callbacks->onoff_cbs.set_cb(p_server,
                                                            &p_rx_msg->meta_data,
                                                            &in_data,
                                                            (p_rx_msg->length == GENERIC_ONOFF_SET_MINLEN) ? NULL : &in_data_tr,
                                                            (p_rx_msg->opcode.opcode == GENERIC_ONOFF_OPCODE_SET) ? &out_data : NULL);

            if (p_rx_msg->opcode.opcode == GENERIC_ONOFF_OPCODE_SET)
            {
                core_tx_type_set(CORE_TX_TYPE_GATT_RELAY);
                (void) status_send(p_server, p_rx_msg, &out_data);
            }
        }
    }
}
/**
 * @brief  Check the received the message.
 * @param  p_rx_msg received message.
 * @param  ADC_FLAG specifies the flag to check.
 * @return true if valid, otherwise false.
 */
static inline bool get_params_validate(const access_message_rx_t * p_rx_msg)
{
    return (p_rx_msg->length == 0);
}
/**
 * @brief  Handle the onoff get message.
 * @param  model_handle model handle.
 * @param  p_rx_msg received message. 
 * @param  p_args arguments pointer for the handler.
 */
static void handle_get(access_model_handle_t model_handle, const access_message_rx_t * p_rx_msg, void * p_args)
{
    generic_onoff_server_t * p_server = (generic_onoff_server_t *) p_args;
    generic_onoff_status_params_t out_data = {0};

    if (get_params_validate(p_rx_msg))
    {
        p_server->settings.p_callbacks->onoff_cbs.get_cb(p_server, &p_rx_msg->meta_data, &out_data);
        (void) status_send(p_server, p_rx_msg, &out_data);
    }
}
/** model handlers */
static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_SIG(GENERIC_ONOFF_OPCODE_SET), handle_set},
    {ACCESS_OPCODE_SIG(GENERIC_ONOFF_OPCODE_SET_UNACKNOWLEDGED), handle_set},
    {ACCESS_OPCODE_SIG(GENERIC_ONOFF_OPCODE_GET), handle_get},
};


/**
 * @brief  Generic onoff server initialization.
 * @param  p_server Pointer to generic_onoff_server_t context.
 * @param  element_index specified element.
 * @return error code.
 */
uint32_t generic_onoff_server_init(generic_onoff_server_t * p_server, uint8_t element_index)
{
    if (p_server == NULL ||
        p_server->settings.p_callbacks == NULL ||
        p_server->settings.p_callbacks->onoff_cbs.set_cb == NULL ||
        p_server->settings.p_callbacks->onoff_cbs.get_cb == NULL )
    {
        return MESH_ERROR_NULL;
    }

    access_model_add_params_t init_params =
    {
        .model_id = ACCESS_MODEL_SIG(GENERIC_ONOFF_SERVER_MODEL_ID),
        .element_index =  element_index,
        .p_opcode_handlers = &m_opcode_handlers[0],
        .opcode_count = ARRAY_SIZE(m_opcode_handlers),
        .p_args = p_server,
        .publish_timeout_cb = periodic_publish_cb
    };

    uint32_t status = access_model_add(&init_params, &p_server->model_handle);

    if (status == MESH_SUCCESS)
    {
        status = access_model_subscription_list_alloc(p_server->model_handle);
    }

    return status;
}

/**
 * @brief  p_server Pointer to generic_onoff_server_t context.
 * @param  p_params parameters related to the onoff status.
 * @return error code.
 */
uint32_t generic_onoff_server_status_publish(generic_onoff_server_t * p_server, const generic_onoff_status_params_t * p_params)
{
    if (p_server == NULL ||
        p_params == NULL)
    {
        return MESH_ERROR_NULL;
    }

    return status_send(p_server, NULL, p_params);
}
