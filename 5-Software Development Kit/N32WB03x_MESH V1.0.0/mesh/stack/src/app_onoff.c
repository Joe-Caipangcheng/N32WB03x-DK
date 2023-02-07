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
 * @file app_onoff.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "app_onoff.h"
#include <stdio.h>
#include <stdint.h>
#include "mesh_error.h"
#include "generic_onoff_server.h"
#include "utils.h"
#include "mesh_common.h"
#include <stdint.h>
#include <stdbool.h>
#include "mesh.h"
#include "net_packet.h"
#include "mesh_list.h"







static void generic_onoff_state_get_cb(const generic_onoff_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       generic_onoff_status_params_t * p_out);
static void generic_onoff_state_set_cb(const generic_onoff_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       const generic_onoff_set_params_t * p_in,
                                       const model_transition_t * p_in_transition,
                                       generic_onoff_status_params_t * p_out);

const generic_onoff_server_callbacks_t onoff_srv_cbs =
{
    .onoff_cbs.set_cb = generic_onoff_state_set_cb,
    .onoff_cbs.get_cb = generic_onoff_state_get_cb
};
/**
 * @brief  Handle the time related during transition.
 * @param  p_server Pointer to app_onoff_server_t context.
 */
static void onoff_state_process_timing(app_onoff_server_t * p_server)
{
    uint32_t status = MESH_SUCCESS;
    if (status != MESH_SUCCESS)
    {

    }
}
/**
 * @brief  Update the onoff state. Call it to actually change the state value.
 * @param  p_server Pointer to app_onoff_server_t context. 
 */
static void onoff_state_value_update(app_onoff_server_t * p_server)
{

    p_server->state.present_onoff = p_server->state.target_onoff;

    generic_onoff_status_params_t status_params;
    status_params.present_on_off = p_server->state.present_onoff;
    status_params.target_on_off = p_server->state.target_onoff;
    status_params.remaining_time_ms = p_server->state.remaining_time_ms;
    (void) generic_onoff_server_status_publish(&p_server->server, &status_params);

    if (!p_server->value_updated)
    {
        p_server->onoff_set_cb(p_server, p_server->state.present_onoff);
        p_server->value_updated = true;
    }
    

}
/**
 * @brief  Call it during transtition. Currently don't use it.
 * @param  p_context context pointer for the timer callback
 */
static void onoff_state_timer_cb(void * p_context)
{
    app_onoff_server_t * p_server = (app_onoff_server_t *) p_context;
    if (p_server->state.delay_ms != 0)
    {
        p_server->state.delay_ms = 0;
        onoff_state_value_update(p_server);
    }
    else if (p_server->state.remaining_time_ms != 0)
    {
    }
    onoff_state_process_timing(p_server);
}
/**
 * @brief  Generic OnOff model interface callback. Use it to get the current state value.
 * @param  p_self generic onoff server model instance.
 * @param  p_meta materials about the incoming message
 * @param  p_out output including the onoff state value
 */
static void generic_onoff_state_get_cb(const generic_onoff_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       generic_onoff_status_params_t * p_out)
{
//    printf("msg: GET \n");
    app_onoff_server_t   * p_server = PARENT_BY_FIELD_GET(app_onoff_server_t, server, p_self);
    p_server->onoff_get_cb(p_server, &p_server->state.present_onoff);
    p_out->present_on_off = p_server->state.present_onoff;
    p_out->target_on_off = p_server->state.target_onoff;
}
/**
 * @brief  Generic OnOff model interface callback. Use it to chang the local onoff state.
 * @param  p_self generic onoff server model instance.
 * @param  p_meta materials about the incoming message
 * @param  p_in onoff state to be present
 * @param  p_out onoff state output
 */
static void generic_onoff_state_set_cb(const generic_onoff_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       const generic_onoff_set_params_t * p_in,
                                       const model_transition_t * p_in_transition,
                                       generic_onoff_status_params_t * p_out)
{
    app_onoff_server_t   * p_server = PARENT_BY_FIELD_GET(app_onoff_server_t, server, p_self);
    p_server->value_updated = false;
    p_server->state.target_onoff = p_in->on_off;
    if (p_in_transition == NULL)
    {
        p_server->state.delay_ms = 0;
        p_server->state.remaining_time_ms = 0;
    }
    else
    {
        p_server->state.delay_ms = p_in_transition->delay_ms;
        p_server->state.remaining_time_ms = p_in_transition->transition_time_ms;
    }

    onoff_state_value_update(p_server);
    onoff_state_process_timing(p_server);
    if (p_out != NULL)
    {
        p_out->present_on_off = p_server->state.present_onoff;
        p_out->target_on_off = p_server->state.target_onoff;
        p_out->remaining_time_ms = p_server->state.remaining_time_ms;
    }
}
/**
 * @brief	Pressing button 1/2 on the Development Kit will result in LED state to trigger
			the STATUS message to inform client about the state change. This is a demonstration of
			state change publication due to local event. 
 * @param  p_server Pointer to app_onoff_server_t context.
*/
void app_onoff_status_publish(app_onoff_server_t * p_server)
{
    p_server->onoff_get_cb(p_server, &p_server->state.present_onoff);
    p_server->state.target_onoff = p_server->state.present_onoff;
    p_server->state.delay_ms = 0;
    p_server->state.remaining_time_ms = 0;
    generic_onoff_status_params_t status = {
                .present_on_off = p_server->state.present_onoff,
                .target_on_off = p_server->state.target_onoff,
                .remaining_time_ms = p_server->state.remaining_time_ms
            };
    (void) generic_onoff_server_status_publish(&p_server->server, &status);
}
/**
 * @brief  Instantiate onoff server on element index.
 * @param  p_server Pointer to app_onoff_server_t context.
 * @param  element_index element that onoff server is located on.
 */
uint32_t app_onoff_init(app_onoff_server_t * p_server, uint8_t element_index)
{
    uint32_t status = MESH_ERROR_INTERNAL;

    if (p_server == NULL)
    {
        return MESH_ERROR_NULL;
    }

    p_server->server.settings.p_callbacks = &onoff_srv_cbs;
    if (p_server->onoff_set_cb == NULL || p_server->onoff_get_cb == NULL)
    {
        return MESH_ERROR_NULL;
    }

    status = generic_onoff_server_init(&p_server->server, element_index);
    if (status == MESH_SUCCESS)
    {

    }
    return status;
}
