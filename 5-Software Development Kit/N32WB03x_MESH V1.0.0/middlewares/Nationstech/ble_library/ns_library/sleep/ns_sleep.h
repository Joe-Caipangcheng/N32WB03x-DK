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
 * @file ns_sleep.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** @addtogroup 
 * @{
 */
#ifndef __NS_SLEEP_H__
#define __NS_SLEEP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes -----------------------------------------------------------------*/
#include <stdint.h>
/* Public typedef -----------------------------------------------------------*/
/* Public define ------------------------------------------------------------*/    

/* Public constants ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/

/**
 * @brief  sleep task function, usually run after rwip_schedule function in main loop.
 * @param  
 * @return 
 * @note   
 */
void ns_sleep(void);

/**
 * @brief  Acquire a sleep lock, it will prevent the os enter sleep mode. 
 *         We should call ns_sleep_lock_release function when this lock can be release. 
 * @param  
 * @return 
 * @note   
 */
uint8_t ns_sleep_lock_acquire(void);

/**
 * @brief  Release a sleep lock, if all the lock has been released, os will enter sleep mode 
 *         when run out of task.
 * @param  
 * @return 
 * @note   
 */
uint8_t ns_sleep_lock_release(void);
/**
 * @brief  User code beofre enter sleep mode
 * @param  
 * @return 
 * @note   
 */
void app_sleep_prepare_proc(void);

/**
 * @brief  User code after out of sleep mode. This function run after interrupt
 *         handler function if any interrupt pending when sleep.
 * @param  
 * @return 
 * @note   
 */
void app_sleep_resume_proc(void);
void bb_reset(void);


#endif /* __NS_SLEEP_H__ */
/**
 * @}
 */


