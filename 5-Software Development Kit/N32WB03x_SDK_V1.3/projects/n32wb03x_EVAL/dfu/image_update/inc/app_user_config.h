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
 * @file app_user_config.h
 * @author Nations Firmware Team
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef _APP_USER_CONFIG_H_
#define _APP_USER_CONFIG_H_

#include "ns_adv_data_def.h"

/* Device name */
#define CUSTOM_DEVICE_NAME                  "ImageUpdate"
#define CUSTOM_BLE_MAC_ADDRESS              "\x8A\x22\x77\x44\x55\x66"


/* adv configer*/
#define CUSTOM_ADV_FAST_INTERVAL               64                                     /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 40 ms.). */
#define CUSTOM_ADV_SLOW_INTERVAL               64                                     /**< Slow advertising interval (in units of 0.625 ms. This value corresponds to 40 ms.). */

#define CUSTOM_ADV_FAST_DURATION               0//30                                         /**< The advertising duration of fast advertising in units of 1 seconds. maximum is 655 seconds */
#define CUSTOM_ADV_SLOW_DURATION               180                                        /**< The advertising duration of slow advertising in units of 1 seconds. maximum is 655 seconds */


#define CUSTOM_USER_ADVERTISE_DATA \
            "\x03"\
            ADV_TYPE_SERVICE_DATA_16BIT_UUID\
            ADV_UUID_DEVICE_INFORMATION_SERVICE\


#define CUSTOM_USER_ADVERTISE_DATA_LEN (sizeof(CUSTOM_USER_ADVERTISE_DATA)-1)

/// Scan response data
#define CUSTOM_USER_ADV_SCNRSP_DATA  \
            "\x0a"\
            ADV_TYPE_MANUFACTURER_SPECIFIC_DATA\
            "\xff\xffNations"

/// Scan response data length- maximum 31 bytes
#define CUSTOM_USER_ADV_SCNRSP_DATA_LEN (sizeof(CUSTOM_USER_ADV_SCNRSP_DATA)-1)


/*  connection config  */
#define MIN_CONN_INTERVAL                   7.5                                        /**< Minimum connection interval (7.5 ms) */
#define MAX_CONN_INTERVAL                   30                                         /**< Maximum connection interval (30 ms). */
#define SLAVE_LATENCY                       6                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    5000                                       /**< Connection supervisory timeout (5000ms). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      (0)                                        /**<  Time of initiating event to update connection params (5 seconds). */

//forfiles config
#define CFG_APP_NS_IUS     1


/* User config  */

#define NS_LOG_ERROR_ENABLE      1
#define NS_LOG_WARNING_ENABLE    1
#define NS_LOG_INFO_ENABLE       1
#define NS_LOG_DEBUG_ENABLE      0

#define NS_LOG_LPUART_ENABLE     0
#define NS_LOG_USART_ENABLE      0
#define NS_LOG_RTT_ENABLE        0

#define NS_TIMER_ENABLE          0

#define FIRMWARE_VERSION            "0.0.01"
#define HARDWARE_VERSION            "0.0.01"


#endif // _APP_USER_CONFIG_H_

