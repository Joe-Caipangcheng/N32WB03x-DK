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
 * @file ns_log_lpuart.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

 /** @addtogroup 
 * @{
 */

#ifndef __NS_LOG_LPUART_H__
#define __NS_LOG_LPUART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
/* Public define ------------------------------------------------------------*/
/* Public typedef -----------------------------------------------------------*/
/* Public define ------------------------------------------------------------*/
#define NS_LOG_LPUART_OUTPUT(color, ...) \
do{                                      \
    if(PRINTF_COLOR_ENABLE)              \
    {                                    \
        printf("%s",color );             \
    }                                    \
    printf(__VA_ARGS__);                 \
}while(0)

    
#define NS_LOG_LPUART_INIT()           \
do{                                    \
    ns_log_lpuart_init();              \
}while(0)

#define NS_LOG_LPUART_DEINIT()        \
do{                                   \
    ns_log_lpuart_deinit();           \
}while(0)
/* Public constants ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
void ns_log_lpuart_init(void);
void ns_log_lpuart_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* __NS_LOG_LPUART_H__ */
/**
 * @}
 */
