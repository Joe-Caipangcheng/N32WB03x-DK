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
 * @file model_common.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "model_common.h"

#include <stdint.h>
#include "mesh_error.h"
#include "mesh_config_core.h"
#include "utils.h"

#define TRANSITION_TIME_STEP_RESOLUTION_100MS   (0x00)
#define TRANSITION_TIME_STEP_RESOLUTION_1S      (0x40)
#define TRANSITION_TIME_STEP_RESOLUTION_10S     (0x80)
#define TRANSITION_TIME_STEP_RESOLUTION_10M     (0xC0)

#define TRANSITION_TIME_STEP_MASK               (0xC0)
#define TRANSITION_TIME_STEP_100MS_FACTOR       (100)
#define TRANSITION_TIME_STEP_1S_FACTOR          (1000)
#define TRANSITION_TIME_STEP_10S_FACTOR         (10*1000)
#define TRANSITION_TIME_STEP_10M_FACTOR         (10*60*1000)


#define TID_VALIDATION_INTERVAL_US              (SEC_TO_US(6))
#define APP_TIMER_MAX_TIMEOUT                   (MODEL_TIMER_MAX_TIMEOUT_TICKS - (APP_TIMER_MIN_TIMEOUT_TICKS * 2))

/**
 * @brief  Update the time during transition
 * @param  p_timer model timer 
 * @return error code
 */
static uint32_t timeout_update_and_schedule(model_timer_t * p_timer)
{
    uint32_t status = MESH_SUCCESS;



    return status;
}
/**
 * @brief  Callback for the model timer
 * @param  p_context context pointer for the timer callback
 */
static void model_timer_cb(void * p_context)
{
    model_timer_t * p_timer = (model_timer_t *) p_context;

    if (p_timer->remaining_ticks == 0)
    {
        p_timer->cb_active = true;
        p_timer->cb(p_timer->p_context);
        p_timer->cb_active = false;

        if (p_timer->mode == MODEL_TIMER_MODE_REPEATED)
        {
            p_timer->remaining_ticks = p_timer->timeout_rtc_ticks;
        }
    }
    if (p_timer->remaining_ticks > 0)
    {
        (void) timeout_update_and_schedule(p_timer);
    }
}
/**
 * @brief  callback for the tid timer
 * @param  timestamp tiem stamp
 * @param  p_context context pointer for the timer callback
 */
static void model_tid_timer_cb(timestamp_t timestamp, void * p_context)
{
    tid_tracker_t * p_item = (tid_tracker_t *) p_context;
    p_item->tid_expiry_timer.cb = NULL;
}
/**
 * @brief  Check whether the received message is new or not.
 * @param  p_tid_tracker pointer to the tid related structure of the received packet
 * @param  p_meta materials related to the received message
 * @param  message_id id of the received message
 * @param  tid tid of the received message
 * @return true if new, otherwise false.
 */
bool model_tid_validate(tid_tracker_t * p_tid_tracker, const access_message_rx_meta_t * p_meta,
                        uint32_t message_id, uint8_t tid)
{
    if (p_tid_tracker->src != p_meta->src.value ||
        p_tid_tracker->dst != p_meta->dst.value ||
        p_tid_tracker->old_tid != tid ||
        p_tid_tracker->message_id != message_id ||
        p_tid_tracker->tid_expiry_timer.cb == NULL)
    {
        p_tid_tracker->src = p_meta->src.value;
        p_tid_tracker->dst = p_meta->dst.value;
        p_tid_tracker->message_id = message_id;
        p_tid_tracker->old_tid = tid;

        p_tid_tracker->tid_expiry_timer.interval = 0;
        p_tid_tracker->tid_expiry_timer.cb = model_tid_timer_cb;
        p_tid_tracker->tid_expiry_timer.p_context = p_tid_tracker;
        p_tid_tracker->new_transaction = true;
    }
    else
    {
        p_tid_tracker->new_transaction = false;
    }

    return p_tid_tracker->new_transaction;
}
/**
 * @brief  Check the tid of the message
 * @return true if new, otherwise false
 */
bool model_transaction_is_new(tid_tracker_t * p_tid_tracker)
{
    return p_tid_tracker->new_transaction;
}
/**
 * @brief  Decode the transition time
 * @return decoded time
 */
uint32_t model_transition_time_decode(uint8_t enc_transition_time)
{
    uint32_t time = 0;

    if ((enc_transition_time & ~TRANSITION_TIME_STEP_MASK) == TRANSITION_TIME_UNKNOWN)
    {
        return MODEL_TRANSITION_TIME_UNKNOWN;
    }

    switch(enc_transition_time & TRANSITION_TIME_STEP_MASK)
    {
        case TRANSITION_TIME_STEP_RESOLUTION_100MS:
            time = (enc_transition_time & ~TRANSITION_TIME_STEP_MASK) * TRANSITION_TIME_STEP_100MS_FACTOR;
        break;

        case TRANSITION_TIME_STEP_RESOLUTION_1S:
            time = (enc_transition_time & ~TRANSITION_TIME_STEP_MASK) * TRANSITION_TIME_STEP_1S_FACTOR;
        break;

        case TRANSITION_TIME_STEP_RESOLUTION_10S:
            time = (enc_transition_time & ~TRANSITION_TIME_STEP_MASK) * TRANSITION_TIME_STEP_10S_FACTOR;
        break;

        case TRANSITION_TIME_STEP_RESOLUTION_10M:
            time = (enc_transition_time & ~TRANSITION_TIME_STEP_MASK) * TRANSITION_TIME_STEP_10M_FACTOR;
        break;

        default:
            break;
    }

    return time;
}

/**
 * @brief  Encode transition time value
 * @param  transition_time transition time
 * @return encoded time
 */
uint8_t model_transition_time_encode(uint32_t transition_time)
{
    uint8_t enc_time = TRANSITION_TIME_UNKNOWN;

    if(transition_time <= TRANSITION_TIME_STEP_100MS_MAX)
    {
        enc_time = (transition_time / TRANSITION_TIME_STEP_100MS_FACTOR) | TRANSITION_TIME_STEP_RESOLUTION_100MS;
    }
    else if (transition_time <= TRANSITION_TIME_STEP_1S_MAX)
    {
        enc_time = (transition_time / TRANSITION_TIME_STEP_1S_FACTOR) | TRANSITION_TIME_STEP_RESOLUTION_1S;
    }
    else if (transition_time <= TRANSITION_TIME_STEP_10S_MAX)
    {
        enc_time = (transition_time / TRANSITION_TIME_STEP_10S_FACTOR) | TRANSITION_TIME_STEP_RESOLUTION_10S;
    }
    else if (transition_time <= TRANSITION_TIME_STEP_10M_MAX)
    {
        enc_time = (transition_time / TRANSITION_TIME_STEP_10M_FACTOR) | TRANSITION_TIME_STEP_RESOLUTION_10M;
    }

    return enc_time;
}
/**
 * @brief  Check whether the input time is valid.
 * @param  enc_transition_time  time checked
 * @return true if valid, otherwise false
 */
bool model_transition_time_is_valid(uint8_t enc_transition_time)
{
    return ((enc_transition_time & ~TRANSITION_TIME_STEP_MASK) != TRANSITION_TIME_UNKNOWN);
}
/**
 * @brief  Get the actual delay
 * @return Actual delay time
 */
uint32_t model_delay_decode(uint8_t enc_delay)
{
    return (enc_delay * DELAY_TIME_STEP_FACTOR_MS);
}
/**
 * @brief  Get delay steps
 * @param  delay delay time 
 * @return steps related to delay
 */
uint8_t model_delay_encode(uint32_t delay)
{
    if (delay > DELAY_TIME_STEP_MAX)
    {
        return DELAY_TIME_STEP_MAX;
    }
    else
    {
        return (delay/DELAY_TIME_STEP_FACTOR_MS);
    }
}
/**
 * @brief  Get the elapsed ticks of the model timer
 * @param  p_timer pointer to the model timer instance
 * @return the elapsed ticks
 */
uint64_t model_timer_elapsed_ticks_get(model_timer_t * p_timer)
{
    return (p_timer->total_rtc_ticks);
}
/**
 * @brief  tiemr schedule
 * @param  p_timer pointer to the model timer instance
 * @return erroe code
 */
uint32_t model_timer_schedule(model_timer_t * p_timer)
{

    if (p_timer == NULL || p_timer->cb == NULL)
    {
        return MESH_ERROR_NULL;
    }

//    if (p_timer->timeout_rtc_ticks < APP_TIMER_MIN_TIMEOUT_TICKS)
//    {
//        return MESH_ERROR_INVALID_PARAM;
//    }

//    (void) app_timer_stop(*p_timer->p_timer_id);

    p_timer->remaining_ticks = p_timer->timeout_rtc_ticks;
//    p_timer->last_rtc_stamp = app_timer_cnt_get();
    p_timer->total_rtc_ticks = 0;

    if (!p_timer->cb_active)
    {
//        return timeout_update_and_schedule(p_timer);
    }

    return MESH_SUCCESS;
}
/**
 * @brief  abort ongoing model timer
 * @param  p_timer pointer to the model timer instance
 */
void model_timer_abort(model_timer_t * p_timer)
{
    p_timer->remaining_ticks = 0;
    p_timer->timeout_rtc_ticks = 0;
//    (void) app_timer_stop(*p_timer->p_timer_id);
}
/**
 * @brief  Create model timer for transition.
 * @param  p_timer pointer to the timer instance
 * @return error code
 */
uint32_t model_timer_create(model_timer_t * p_timer)
{
    if (p_timer == NULL || p_timer->cb == NULL)
    {
        return MESH_ERROR_NULL;
    }

    p_timer->cb_active = false;
//    return (app_timer_create(p_timer->p_timer_id, APP_TIMER_MODE_SINGLE_SHOT, model_timer_cb));
    return MESH_SUCCESS;
}
