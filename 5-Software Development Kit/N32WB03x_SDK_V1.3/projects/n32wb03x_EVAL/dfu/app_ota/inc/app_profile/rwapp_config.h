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
 * @file rwapp_config.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */


#ifndef _RWAPP_CONFIG_H_
#define _RWAPP_CONFIG_H_

/**
 * @addtogroup app
 * @brief Application configuration definition
 *
 * @{
 **/

#include "app_user_config.h"
#include "rwip_config.h"
/* Includes ------------------------------------------------------------------*/

/* Define ------------------------------------------------------------*/


/// Application Profile
#if defined(CFG_APP_PRF)
#define BLE_APP_PRF          1
#else // defined(CFG_APP_PRF)
#define BLE_APP_PRF          0
#endif // defined(CFG_APP_PRF)


/// Health Thermometer Application
#if defined(CFG_APP_HT)
#define BLE_APP_HT           1
#else // defined(CFG_APP_HT)
#define BLE_APP_HT           0
#endif // defined(CFG_APP_HT)

/// HID Application
#if defined(CFG_APP_HID)
#define BLE_APP_HID          1
#else // defined(CFG_APP_HID)
#define BLE_APP_HID          0
#endif // defined(CFG_APP_HID)

/// DIS Application
#if defined(CFG_APP_DIS)
#define BLE_APP_DIS          1
#else // defined(CFG_APP_DIS)
#define BLE_APP_DIS          0
#endif // defined(CFG_APP_DIS)



/// Battery Service Application
#if (CFG_APP_BATT)
#define BLE_APP_BATT         1
#else
#define BLE_APP_BATT         0
#endif //(CFG_APP_BATT)

/// Security Application
#if (defined(CFG_APP_SEC) || BLE_APP_HID)
#define BLE_APP_SEC          1
#else //(defined(CFG_APP_SEC) || BLE_APP_HID)
#define BLE_APP_SEC          0
#endif //(defined(CFG_APP_SEC) || BLE_APP_HID)


#define AM0_APP_OPTIONAL_CHARACTERISTICS        0



#if defined(CFG_APP_NS_IUS)
#define BLE_APP_NS_IUS          1
#else // defined(CFG_APP_NS_IUS)
#define BLE_APP_NS_IUS          0
#endif // defined(CFG_APP_NS_IUS)


/// @} rwapp_config

#endif /* _RWAPP_CONFIG_H_ */
