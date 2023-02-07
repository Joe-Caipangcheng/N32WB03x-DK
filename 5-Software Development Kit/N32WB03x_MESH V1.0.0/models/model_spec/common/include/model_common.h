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
 * @file model_common.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef MODEL_COMMON_H__
#define MODEL_COMMON_H__

#include <stdint.h>
#include "utils.h"
#include "timer_scheduler.h"
#include "access.h"


#define MODEL_TRANSITION_TIME_UNKNOWN               (UINT32_MAX)

#ifndef MODEL_ACKNOWLEDGED_TRANSACTION_TIMEOUT
#define MODEL_ACKNOWLEDGED_TRANSACTION_TIMEOUT      (SEC_TO_US(30))
#endif

#define TRANSITION_TIME_STEP_100MS_MAX          (6200ul)
#define TRANSITION_TIME_STEP_1S_MAX             (SEC_TO_MS(62ul))
#define TRANSITION_TIME_STEP_10S_MAX            (SEC_TO_MS(620ul))
#define TRANSITION_TIME_STEP_10M_MAX            (MIN_TO_MS(620ul))
#define TRANSITION_TIME_MAX                     (0x3E)
#define TRANSITION_TIME_UNKNOWN                 (0x3F)
#define TRANSITION_TIME_MAX_MS                  (TRANSITION_TIME_STEP_10M_MAX)

#define DELAY_TIME_STEP_FACTOR_MS               (5)
#define DELAY_TIME_STEP_MAX                     (0xFF)
#define DELAY_TIME_MAX_MS                       (DELAY_TIME_STEP_MAX * DELAY_TIME_STEP_FACTOR_MS)

#define MODEL_TIMER_TIMEOUT_MIN_TICKS           (APP_TIMER_TICKS(1))
#define MODEL_TIMER_TIMEOUT_MIN_US              (MODEL_TIMER_PERIOD_US_GET(MODEL_TIMER_TIMEOUT_MIN_TICKS))
#define MODEL_TIMER_MAX_TIMEOUT_TICKS           (APP_TIMER_MAX_CNT_VAL)
#define MODEL_TIMER_PERIOD_MS_GET(TICKS)        ((1000ul * (TICKS)) / APP_TIMER_CLOCK_FREQ)
#define MODEL_TIMER_PERIOD_US_GET(TICKS)        ((1000ul * 1000ul * (TICKS)) / APP_TIMER_CLOCK_FREQ)
#define MODEL_TIMER_TICKS_GET_MS(MS_TIME)                                           \
            ((uint64_t)ROUNDED_DIV((MS_TIME) * (uint64_t)APP_TIMER_CLOCK_FREQ,      \
             1000 * (APP_TIMER_CONFIG_RTC_FREQUENCY + 1)))

#define MODEL_TIMER_TICKS_GET_US(US_TIME)                                           \
            ((uint64_t)ROUNDED_DIV(                                                 \
            (US_TIME) * (uint64_t)APP_TIMER_CLOCK_FREQ,                             \
            1000 * 1000 * (APP_TIMER_CONFIG_RTC_FREQUENCY + 1)))

typedef struct
{
    uint32_t transition_time_ms;                            /**< Transition time value in milliseconds */
    uint32_t delay_ms;                                         /**< Message execution delay in milliseconds */
} model_transition_t;

typedef struct
{
    uint16_t src;
    uint16_t dst;
    uint32_t message_id;
    uint8_t old_tid;
    bool new_transaction;
    timer_event_t tid_expiry_timer;
} tid_tracker_t;

typedef enum
{
    MODEL_TIMER_MODE_SINGLE_SHOT,
    MODEL_TIMER_MODE_REPEATED
} model_timer_mode_t;

typedef void(*model_timer_cb_t)(void * p_context);

typedef struct
{
    model_timer_mode_t mode;
    uint64_t timeout_rtc_ticks;
    void * p_context;
    model_timer_cb_t cb;
    uint64_t total_rtc_ticks;
    uint64_t remaining_ticks;
    uint32_t last_rtc_stamp;
    bool cb_active;
} model_timer_t;

/**
 * @brief  Decode the transition time
 * @return decoded time
 */
uint32_t model_transition_time_decode(uint8_t enc_transition_time);

/**
 * @brief  Encode transition time value
 * @param  transition_time transition time
 * @return encoded time
 */
uint8_t model_transition_time_encode(uint32_t transition_time);

/**
 * @brief  Check whether the input time is valid.
 * @param  enc_transition_time  time checked
 * @return true if valid, otherwise false
 */
bool model_transition_time_is_valid(uint8_t enc_transition_time);

/**
 * @brief  Get the actual delay
 * @return Actual delay time
 */
uint32_t model_delay_decode(uint8_t enc_delay);

/**
 * @brief  Get delay steps
 * @param  delay delay time 
 * @return steps related to delay
 */
uint8_t model_delay_encode(uint32_t delay);

/**
 * @brief  Check whether the received message is new or not.
 * @param  p_tid_tracker pointer to the tid related structure of the received packet
 * @param  p_meta materials related to the received message
 * @param  message_id id of the received message
 * @param  tid tid of the received message
 * @return true if new, otherwise false.
 */
bool model_tid_validate(tid_tracker_t * p_tid_tracker, const access_message_rx_meta_t * p_meta,
                        uint32_t message_id, uint8_t tid);

/**
 * @brief  Check the tid of the message
 * @return true if new, otherwise false
 */
bool model_transaction_is_new(tid_tracker_t * p_tid_tracker);

/**
 * @brief  tiemr schedule
 * @param  p_timer pointer to the model timer instance
 * @return erroe code
 */
uint32_t model_timer_schedule(model_timer_t * p_timer);

/**
 * @brief  abort ongoing model timer
 * @param  p_timer pointer to the model timer instance
 */
void model_timer_abort(model_timer_t * p_timer);

/**
 * @brief  Get the elapsed ticks of the model timer
 * @param  p_timer pointer to the model timer instance
 * @return the elapsed ticks
 */
uint64_t model_timer_elapsed_ticks_get(model_timer_t * p_timer);

/**
 * @brief  Create model timer for transition.
 * @param  p_timer pointer to the timer instance
 * @return error code
 */
uint32_t model_timer_create(model_timer_t * p_timer);


#endif /* MODEL_COMMON_H__ */

