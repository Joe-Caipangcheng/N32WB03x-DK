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
 * @file generic_onoff_client.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "generic_onoff_client.h"
#include "model_common.h"
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
 * @brief  Handle the onoff status message.
 * @param  model_handle model handle.
 * @param  p_rx_msg received message. 
 * @param  p_args arguments pointer for the handler.
 */
static void status_handle(access_model_handle_t handle, const access_message_rx_t * p_rx_msg, void * p_args)
{
//    printf("status_handle\r\n");
    generic_onoff_client_t * p_client = (generic_onoff_client_t *) p_args;
    generic_onoff_status_params_t in_data = {0};

    if (p_rx_msg->length == GENERIC_ONOFF_STATUS_MINLEN || p_rx_msg->length == GENERIC_ONOFF_STATUS_MAXLEN)
    {
        generic_onoff_status_msg_pkt_t * p_msg_params_packed = (generic_onoff_status_msg_pkt_t *) p_rx_msg->p_data;

        if (p_rx_msg->length == GENERIC_ONOFF_STATUS_MINLEN)
        {
            in_data.present_on_off = p_msg_params_packed->present_on_off;
            in_data.target_on_off = p_msg_params_packed->present_on_off;
            in_data.remaining_time_ms = 0;
        }
        else
        {
            in_data.present_on_off = p_msg_params_packed->present_on_off;
            in_data.target_on_off = p_msg_params_packed->target_on_off;
            in_data.remaining_time_ms = model_transition_time_decode(p_msg_params_packed->remaining_time);
        }

        p_client->settings.p_callbacks->onoff_status_cb(p_client, &p_rx_msg->meta_data, &in_data);
    }
}
/** model handlers */
static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_SIG(GENERIC_ONOFF_OPCODE_STATUS), status_handle},
};
/**
 * @brief  Set the onoff client model publishement parameters
 * @param  p_set pointer to the sent packet structure.
 * @param  p_params message related parameters.
 * @param  p_transition parameters related to transition
 * @return onoff set message length.
 */
static uint8_t message_set_packet_create(generic_onoff_set_msg_pkt_t *p_set, const generic_onoff_set_params_t * p_params,
                                      const model_transition_t * p_transition)
{
        p_set->on_off = p_params->on_off ? 1 : 0;
        p_set->tid = p_params->tid;

        if (p_transition != NULL)
        {
            p_set->transition_time = model_transition_time_encode(p_transition->transition_time_ms);
            p_set->delay = model_delay_encode(p_transition->delay_ms);
            return GENERIC_ONOFF_SET_MAXLEN;
        }
        else
        {
            return GENERIC_ONOFF_SET_MINLEN;
        }
}
/**
 * @brief  Create the published message.
 * @param  p_client Pointer to generic_onoff_client_t context.
 * @param  tx_opcode model opcode.
 * @param  p_buffer data which will be sent out
 * @param  length length of the buffer
 * @param  p_message pointer to the message structure
 */
static void message_create(generic_onoff_client_t * p_client, uint16_t tx_opcode, const uint8_t * p_buffer,
                           uint16_t length, access_message_tx_t *p_message)
{
    p_message->opcode.opcode = tx_opcode;
    p_message->opcode.company_id = ACCESS_COMPANY_ID_NONE;
    p_message->p_buffer = p_buffer;
    p_message->length = length;
    p_message->force_segmented = p_client->settings.force_segmented;
    p_message->transmic_size = p_client->settings.transmic_size;
    p_message->access_token = mesh_unique_token_get();
}



/**
 * @brief  Generic onoff client initialization.
 * @param  p_client Pointer to generic_onoff_client_t context.
 * @param  element_index specified element.
 * @return error code.
 */
uint32_t generic_onoff_client_init(generic_onoff_client_t * p_client, uint8_t element_index)
{
    if (p_client->settings.timeout == 0)
    {
        p_client->settings.timeout= MODEL_ACKNOWLEDGED_TRANSACTION_TIMEOUT;
    }

    access_model_add_params_t add_params =
    {
        .model_id = ACCESS_MODEL_SIG(GENERIC_ONOFF_CLIENT_MODEL_ID),
        .element_index = element_index,
        .p_opcode_handlers = &m_opcode_handlers[0],
        .opcode_count = ARRAY_SIZE(m_opcode_handlers),
        .p_args = p_client,
        .publish_timeout_cb = p_client->settings.p_callbacks->periodic_publish_cb
    };

    uint32_t status = access_model_add(&add_params, &p_client->model_handle);

    if (status == MESH_SUCCESS)
    {
        status = access_model_subscription_list_alloc(p_client->model_handle);
    }

    return status;
}


/**
 * @brief  Send out the set message with unack.
 * @param  p_client Pointer to generic_onoff_client_t context.
 * @param  p_params parameters related to the sent message.
 * @param  p_transition parameters related to the transition.
 * @param  repeats number of publishment.
 * @return error code.
 */
uint32_t generic_onoff_client_set_unack(generic_onoff_client_t * p_client, const generic_onoff_set_params_t * p_params,
                                        const model_transition_t * p_transition, uint8_t repeats)
{
    uint32_t status = MESH_ERROR_INTERNAL;
    uint8_t server_msg_length = message_set_packet_create(&p_client->msg_pkt.set, p_params, p_transition);
    message_create(p_client, GENERIC_ONOFF_OPCODE_SET_UNACKNOWLEDGED,
                   (const uint8_t *) &p_client->msg_pkt.set, server_msg_length,
                   &p_client->access_message.message);
    core_tx_type_set(CORE_TX_TYPE_ADV);               
    access_model_publish(p_client->model_handle, &p_client->access_message.message);
    return status;
}

/**
 * @brief  Info for the raliable publishment.
 * @param  p_client Pointer to generic_onoff_client_t context.
 * @param  reply_opcode model opcode for replying message.
 * @param  p_reliable pointer to reliable reply message structure.
 */
static void reliable_context_create(generic_onoff_client_t * p_client, uint16_t reply_opcode,
                                    access_reliable_t * p_reliable)
{
    p_reliable->model_handle = p_client->model_handle;
    p_reliable->reply_opcode.opcode = reply_opcode;
    p_reliable->reply_opcode.company_id = ACCESS_COMPANY_ID_NONE;
    p_reliable->timeout = p_client->settings.timeout;
    p_reliable->status_cb = p_client->settings.p_callbacks->ack_transaction_status_cb;
}
/**
 * @brief  Reliable publishment.
 * @param  p_client Pointer to generic_onoff_client_t context.
 * @param  p_params parameters related to reliable publishment message.
 * @param  p_transition parameters related to transition.
 */
uint32_t generic_onoff_client_set(generic_onoff_client_t * p_client, const generic_onoff_set_params_t * p_params,
                                  const model_transition_t * p_transition)
{
    if (p_client == NULL || p_params == NULL)
    {
        return MESH_ERROR_NULL;
    }

    if (p_transition != NULL &&
        (p_transition->transition_time_ms > TRANSITION_TIME_MAX_MS ||
         p_transition->delay_ms > DELAY_TIME_MAX_MS))
    {
        return MESH_ERROR_INVALID_PARAM;
    }

    if (access_reliable_model_is_free(p_client->model_handle))
    {
        uint8_t server_msg_length = message_set_packet_create(&p_client->msg_pkt.set, p_params, p_transition);
        message_create(p_client, GENERIC_ONOFF_OPCODE_SET, (const uint8_t *) &p_client->msg_pkt.set, server_msg_length, &p_client->access_message.message);
        reliable_context_create(p_client, GENERIC_ONOFF_OPCODE_STATUS, &p_client->access_message);
        core_tx_type_set(CORE_TX_TYPE_ADV);
        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return MESH_ERROR_BUSY;
    }
}


