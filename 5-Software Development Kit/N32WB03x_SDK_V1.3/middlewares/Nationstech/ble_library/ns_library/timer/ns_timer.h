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
 * @file ns_timer.h
 * @author Nations Firmware Team
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** @addtogroup 
 * @{
 */

#ifndef _NS_TIMER_H_
#define _NS_TIMER_H_

/* Includes ------------------------------------------------------------------*/
#include "ke_msg.h"
/* Public typedef -----------------------------------------------------------*/
/// Timer handler type
typedef uint8_t timer_hnd_t;
/// Timer callback function type definition
typedef void (* timer_callback_t)(void);
/* Public define ------------------------------------------------------------*/    
/// Max timer delay 41943sec (41943000ms)
#define KE_TIMER_DELAY_MAX          (41943000)
/// Value indicating an invalide timer operation
#define NS_TIMER_INVALID_HANDLER    (0xFF)
/* Public constants ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/


/**
 * @brief     Process handler for the Timer messages.
 * @param msgid   Id of the message received
 * @param param   Pointer to the parameters of the message
 * @param dest_id ID of the receiving task instance (probably unused)
 * @param src_id  ID of the sending task instance
 * @param msg_ret Result of the message handler
 * @return    Returns if the message is handled by the process handler
 * @note   
 */

enum process_event_response ns_timer_api_process_handler(ke_msg_id_t const msgid,
                                                          void const *param,
                                                          ke_task_id_t const dest_id,
                                                          ke_task_id_t const src_id,
                                                          enum ke_msg_status_tag *msg_ret);


/**
 * @brief  Create a new timer.
 * @param  Param input
 * @brief Create a new timer.
 * @param delay The amount of timer value to wait (time resolution is 1ms)
 * @param fn    The callback to be called when the timer expires
 * @return The handler of the timer for future reference. If there are not timers available
 *         NS_TIMER_INVALID_HANDLER will be returned
 */
timer_hnd_t ns_timer_create(const uint32_t delay, timer_callback_t fn);

/**
 * @brief Cancel an active timer.
 * @param timer_id The timer handler to cancel
 */
void ns_timer_cancel(const timer_hnd_t timer_id);

/**
 * @brief Modify the delay of an existing timer.
 * @param timer_id The timer handler to modify
 * @param delay    The new delay value (time resolution is 1ms)
 * @return The timer handler if everything is ok
 */
timer_hnd_t ns_timer_modify(const timer_hnd_t timer_id, const uint32_t delay);

/**
 * @brief Cancel all the active timers.
 */
void ns_timer_cancel_all(void);

#endif // _NS_TIMER_H_

/**
 * @}
 */
