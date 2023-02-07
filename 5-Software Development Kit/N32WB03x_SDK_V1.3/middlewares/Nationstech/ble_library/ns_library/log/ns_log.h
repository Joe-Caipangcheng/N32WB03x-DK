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
 * @file ns_log.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef __NS_LOG_H__
#define __NS_LOG_H__


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app_user_config.h"

#if   (NS_LOG_LPUART_ENABLE)
#include "ns_log_lpuart.h" 
#elif (NS_LOG_USART_ENABLE)
#include "ns_log_usart.h"
#elif (NS_LOG_RTT_ENABLE)
#include "ns_log_rtt.h"       
#endif

/* Private define ------------------------------------------------------------*/
#ifndef NS_LOG_ERROR_ENABLE
#define NS_LOG_ERROR_ENABLE      0
#endif
#ifndef NS_LOG_WARNING_ENABLE
#define NS_LOG_WARNING_ENABLE    0
#endif
#ifndef NS_LOG_INFO_ENABLE
#define NS_LOG_INFO_ENABLE       0
#endif
#ifndef NS_LOG_DEBUG_ENABLE
#define NS_LOG_DEBUG_ENABLE      0
#endif
#ifndef NS_LOG_LPUART_ENABLE
#define NS_LOG_LPUART_ENABLE     0
#endif
#ifndef NS_LOG_USART_ENABLE
#define NS_LOG_USART_ENABLE      0
#endif
#ifndef NS_LOG_RTT_ENABLE
#define NS_LOG_RTT_ENABLE        0
#endif

#ifndef PRINTF_COLOR_ENABLE
#define PRINTF_COLOR_ENABLE      0
#endif
#define LOG_COLOR_RED            "\033[0;31m"
#define LOG_COLOR_YELLOW         "\033[0;33m"
#define LOG_COLOR_CYAN           "\033[0;36m"
#define LOG_COLOR_GREEN          "\033[0;32m"


#if   (NS_LOG_LPUART_ENABLE)
#define NS_LOG_INTERNAL_OUTPUT(color, ...)  NS_LOG_LPUART_OUTPUT(color, __VA_ARGS__)
#define NS_LOG_INTERNAL_INIT()              NS_LOG_LPUART_INIT() 
#define NS_LOG_INTERNAL_DEINIT()           
#elif (NS_LOG_USART_ENABLE)
#define NS_LOG_INTERNAL_OUTPUT(color, ...)  NS_LOG_USART_OUTPUT(color, __VA_ARGS__)
#define NS_LOG_INTERNAL_INIT()              NS_LOG_USART_INIT()
#define NS_LOG_INTERNAL_DEINIT()            NS_LOG_USART_DEINIT()
#elif (NS_LOG_RTT_ENABLE)
#define NS_LOG_INTERNAL_OUTPUT(color, ...)  NS_LOG_RTT_OUTPUT(color, __VA_ARGS__)
#define NS_LOG_INTERNAL_INIT()              NS_LOG_RTT_INIT()
#define NS_LOG_INTERNAL_DEINIT()            NS_LOG_RTT_DEINIT()
#else
#define NS_LOG_INTERNAL_OUTPUT(color, ...)  
#define NS_LOG_INTERNAL_INIT() 
#define NS_LOG_INTERNAL_DEINIT()
#endif
/* Public typedef -----------------------------------------------------------*/
/* Public define ------------------------------------------------------------*/
#if  NS_LOG_ERROR_ENABLE
#define NS_LOG_ERROR(...)        NS_LOG_INTERNAL_OUTPUT(LOG_COLOR_RED, __VA_ARGS__)
#else
#define NS_LOG_ERROR( ...) 
#endif

#if NS_LOG_WARNING_ENABLE
#define NS_LOG_WARNING(...)      NS_LOG_INTERNAL_OUTPUT(LOG_COLOR_YELLOW, __VA_ARGS__)
#else
#define NS_LOG_WARNING( ...) 
#endif

#if  NS_LOG_INFO_ENABLE
#define NS_LOG_INFO(...)         NS_LOG_INTERNAL_OUTPUT(LOG_COLOR_CYAN, __VA_ARGS__)
#else
#define NS_LOG_INFO( ...) 
#endif

#if  NS_LOG_DEBUG_ENABLE
#define NS_LOG_DEBUG(...)        NS_LOG_INTERNAL_OUTPUT(LOG_COLOR_GREEN, __VA_ARGS__)
#else
#define NS_LOG_DEBUG( ...) 
#endif


/**
 * @brief   Initialization the log module
 * @param  
 * @return 
 * @note   
 */
#define NS_LOG_INIT()              NS_LOG_INTERNAL_INIT()

/**
 * @brief   Deinitialize the log module
 * @param  
 * @return 
 * @note   
 */
#define NS_LOG_DEINIT()            NS_LOG_INTERNAL_DEINIT()
/* Public constants ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/



#ifdef __cplusplus
}
#endif

#endif /* __NS_LOG_H__ */
/**
 * @}
 */
