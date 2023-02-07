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
 * @file template.c
 * @author Nations Firmware Team
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** @addtogroup ns_timer
 * @{
 */

/* Includes ------------------------------------------------------------------*/
#include "rwip_config.h"             // SW configuration

#if (BLE_APP_PRESENT)
#include "ke_timer.h"
#include "ke_msg.h"
#include "ns_ble.h"
#include "ns_timer.h"
#include "ns_ble_task.h"
#include <stdio.h>
/* Private define ------------------------------------------------------------*/
#define NS_TIMER_MAX_NUM                        (NS_TIMER_API_LAST_MES - NS_TIMER_API_MES0 + 1)
//handler: Timer handler values = 0...NS_TIMER_MAX_NUM -1
#define NS_TIMER_HANDLER_TO_MSG_ID(hnd_idx)     (hnd_idx  + NS_TIMER_API_MES0)
#define NS_TIMER_MSG_ID_TO_HANDLER(msg_id)      (msg_id   - NS_TIMER_API_MES0)
#define NS_TIMER_HANDLER_IS_VALID(hnd_idx)      (hnd_idx  < NS_TIMER_MAX_NUM)

#define CREATE_TIMER(timer_id,delay)             ke_timer_set(NS_TIMER_HANDLER_TO_MSG_ID(timer_id), TASK_APP, delay)

/* Private typedef -----------------------------------------------------------*/
struct cancel_timer_t
{
    timer_hnd_t timer_id;
    uint32_t delay;
};

struct modify_timer_t
{
    timer_hnd_t timer_id;
    uint32_t delay;
};
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// Array that holds the callback function of the active timers
static timer_callback_t timer_callback_array[NS_TIMER_MAX_NUM];

// Array that holds the callback function of the active timers, whose delay period is to be modified
static timer_callback_t modified_timer_callback_array[NS_TIMER_MAX_NUM];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
 * @brief Place a callback in the first available position in the timer callback array.
 * @param fn The callback to be added
 * @return The handler of the timer for future reference. NS_TIMER_INVALID_HANDLER if
 * there is no timer available
 */
static timer_hnd_t set_callback(timer_callback_t fn)
{
    for (int i = 0; i < NS_TIMER_MAX_NUM; i++)
    {
        if (timer_callback_array[i] == NULL)
        {
            timer_callback_array[i] = fn;
            return (i);
        }
    }
    return NS_TIMER_INVALID_HANDLER;
}

/**
 * @brief Dummy callback used when canceling a timer.
 */
static void timer_canceled_handler(void)
{
    //keep empty
}

/**
 * @brief Dummy callback used when modifying a timer.
 */
static void timer_modified_handler(void)
{
    //keep empty
}

/**
 * @brief Call the callback of a specific timer handler if it exists.
 * @param timer_id The handler to call
 */
static void call_user_callback(const timer_hnd_t timer_id)
{
    void (*return_timer_cb)(timer_hnd_t timer_id);

    if NS_TIMER_HANDLER_IS_VALID(timer_id)
    {
        if (timer_callback_array[timer_id] != NULL)
        {
            timer_callback_t fn = timer_callback_array[timer_id];
            // Check if the a timer to be modified or canceled has expired
            if ((fn != timer_modified_handler) && (fn != timer_canceled_handler))
            {
                timer_callback_array[timer_id] = NULL;
                modified_timer_callback_array[timer_id] = NULL;
                return_timer_cb = (void (*)(timer_hnd_t))fn;
                return_timer_cb(timer_id);
            }
        }
    }
    else
    {
        ASSERT_WARN(0,0,0);
    }
}

/**
 * @brief Handler function that is called when the TASK_APP receives the APP_CANCEL_TIMER
 *        message.
 * @param msgid Id of the message received
 * @param param The timer details to be canceled
 * @param dest_id ID of the receiving task instance
 * @param src_id ID of the sending task instance
 * @return KE_MSG_CONSUMED
 */
static int cancel_timer_msg_handler(ke_msg_id_t const msgid,
                                struct cancel_timer_t const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    if NS_TIMER_HANDLER_IS_VALID(param->timer_id)
    {
        int i = param->timer_id;
        if (timer_callback_array[i] == timer_canceled_handler)
        {
            timer_callback_array[i] = NULL;
            modified_timer_callback_array[i] = NULL;
        }
        else if (timer_callback_array[i] == timer_modified_handler)
        {
            struct modify_timer_t *req = KE_MSG_ALLOC(APP_MODIFY_TIMER,
                                                           TASK_APP,
                                                           TASK_APP,
                                                           modify_timer_t);

            req->timer_id = param->timer_id;
            req->delay = param->delay;
            ke_msg_send(req);
        }
    }
    else
    {
        ASSERT_ERR(0);
    }
    return KE_MSG_CONSUMED;
}

/**
*
 * @brief Handler function that is called when the TASK_APP receives the APP_MODIFY_TIMER
 *        message.
 * @param msgid Id of the message received
 * @param param The timer details to be modified
 * @param dest_id ID of the receiving task instance
 * @param src_id ID of the sending task instance
 * @return KE_MSG_CONSUMED
*
 */
static int modify_timer_msg_handler(ke_msg_id_t const msgid,
                                struct modify_timer_t const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    if NS_TIMER_HANDLER_IS_VALID(param->timer_id)
    {
        if (timer_callback_array[param->timer_id] == timer_modified_handler)
        {
            // Restore timer callback function
            timer_callback_array[param->timer_id] = modified_timer_callback_array[param->timer_id];

            // Re-create the timer with new delay
            CREATE_TIMER(param->timer_id, param->delay);
        }
    }
    else
    {
        ASSERT_ERR(0);
    }
    return KE_MSG_CONSUMED;
}


enum process_event_response ns_timer_api_process_handler(ke_msg_id_t const msgid,
                                                          void const *param,
                                                          ke_task_id_t const dest_id,
                                                          ke_task_id_t const src_id,
                                                          enum ke_msg_status_tag *msg_ret)
{
    
    NS_LOG_DEBUG("app_timer_api_process_handler:msgid:%04x\r\n",msgid);
    switch (msgid)
    {
        case APP_CANCEL_TIMER:
            *msg_ret = (enum ke_msg_status_tag)cancel_timer_msg_handler(msgid, param, dest_id, src_id);
            return PR_EVENT_HANDLED;

        case APP_MODIFY_TIMER:
            *msg_ret = (enum ke_msg_status_tag)modify_timer_msg_handler(msgid, param, dest_id, src_id);
            return PR_EVENT_HANDLED;

        default:
            if ((msgid < NS_TIMER_API_MES0) || (msgid > NS_TIMER_API_LAST_MES))
            {
                *msg_ret = KE_MSG_NO_FREE;
                return PR_EVENT_UNHANDLED;
            }
            else
            {
                call_user_callback(NS_TIMER_MSG_ID_TO_HANDLER(msgid));
                *msg_ret = KE_MSG_CONSUMED;
            }
            return PR_EVENT_HANDLED;
    }
}

timer_hnd_t ns_timer_create(const uint32_t delay, timer_callback_t fn)
{
    // Sanity checks
    ASSERT_ERR(delay > 0);                  // Delay should not be zero
    ASSERT_ERR(delay < KE_TIMER_DELAY_MAX); // Delay should not be more than maximum allowed

    timer_hnd_t timer_id = set_callback(fn);
    if (timer_id == NS_TIMER_INVALID_HANDLER)
    {
        return NS_TIMER_INVALID_HANDLER; //No timers available
    }

    // Create timer
    CREATE_TIMER(timer_id, delay);

    return timer_id;
}

void ns_timer_cancel(const timer_hnd_t timer_id)
{
    if NS_TIMER_HANDLER_IS_VALID(timer_id)
    {
        if ((timer_callback_array[timer_id] != NULL) &&
            (timer_callback_array[timer_id] != timer_canceled_handler))
        {
            // Remove the timer from the timer queue
            ke_timer_clear(NS_TIMER_HANDLER_TO_MSG_ID(timer_id), TASK_APP);

            timer_callback_array[timer_id] = timer_canceled_handler;

            struct cancel_timer_t *req = KE_MSG_ALLOC(APP_CANCEL_TIMER, TASK_APP, TASK_APP,
                                                           cancel_timer_t);

            req->timer_id = timer_id;
            ke_msg_send(req);
        }
        else
        {
            ASSERT_WARN(0,0,0);
        }
   }
   else
   {
       ASSERT_WARN(0,0,0);
   }
}

timer_hnd_t ns_timer_modify(const timer_hnd_t timer_id, uint32_t delay)
{
    // Sanity checks
    ASSERT_ERR(delay > 0);                  // Delay should not be zero
    ASSERT_ERR(delay < KE_TIMER_DELAY_MAX); // Delay should not be more than maximum allowed

    if NS_TIMER_HANDLER_IS_VALID(timer_id)
    {
        if ((timer_callback_array[timer_id] != NULL) &&
            (timer_callback_array[timer_id] != timer_modified_handler))
        {
            // Remove the timer from the timer queue
            ke_timer_clear(NS_TIMER_HANDLER_TO_MSG_ID(timer_id), TASK_APP);

            // Store the timer function callback
            modified_timer_callback_array[timer_id] = timer_callback_array[timer_id];

            timer_callback_array[timer_id] = timer_modified_handler;

            struct cancel_timer_t *req = KE_MSG_ALLOC(APP_CANCEL_TIMER, TASK_APP, TASK_APP,
                                                           cancel_timer_t);

            req->timer_id = timer_id;
            req->delay = delay;
            ke_msg_send(req);

            return timer_id;
        }
        else
        {
            ASSERT_WARN(0,0,0);
        }
    }
    else
    {
        ASSERT_WARN(0,0,0);
    }
    return NS_TIMER_INVALID_HANDLER;
}

void ns_timer_cancel_all(void)
{
    for (int i = 0; i < NS_TIMER_MAX_NUM; i++)
    {
        if ((timer_callback_array[i] != NULL) &&
            (timer_callback_array[i] != timer_canceled_handler))
            {
                ns_timer_cancel(i);
            }
    }
}

#endif // (BLE_APP_PRESENT)


/**
 * @}
 */
