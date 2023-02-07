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
#define CUSTOM_DEVICE_NAME                  "hid_mouse"

/* adv configer*/
#define CUSTOM_ADV_FAST_INTERVAL               160                                        /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 100 ms.). */
#define CUSTOM_ADV_SLOW_INTERVAL               3200                                       /**< Slow advertising interval (in units of 0.625 ms. This value corresponds to 2 seconds). */

#define CUSTOM_ADV_FAST_DURATION               0//30                                         /**< The advertising duration of fast advertising in units of 1 seconds. maximum is 655 seconds */
#define CUSTOM_ADV_SLOW_DURATION               180                                        /**< The advertising duration of slow advertising in units of 1 seconds. maximum is 655 seconds */


#define CUSTOM_USER_ADVERTISE_DATA \
            "\x03"\
            ADV_TYPE_INCOMPLETE_LIST_16BIT_SERVICE_IDS\
            ADV_UUID_HUMAN_INTERFACE_DEVICE_SERVICE\



#define CUSTOM_USER_ADVERTISE_DATA_LEN (sizeof(CUSTOM_USER_ADVERTISE_DATA)-1)

/// Scan response data
#define CUSTOM_USER_ADV_SCNRSP_DATA  \
            "\x0a"\
            ADV_TYPE_MANUFACTURER_SPECIFIC_DATA\
            "\xff\xffNations"

/// Scan response data length- maximum 31 bytes
#define CUSTOM_USER_ADV_SCNRSP_DATA_LEN (sizeof(CUSTOM_USER_ADV_SCNRSP_DATA)-1)


/*  connection config  */
#define MIN_CONN_INTERVAL                   30                                         /**< Minimum connection interval (30 ms) */
#define MAX_CONN_INTERVAL                   50                                         /**< Maximum connection interval (50 ms). */
#define SLAVE_LATENCY                       5                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    5000                                        /**< Connection supervisory timeout (5000ms). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      0                                          /**<  Time of initiating event to update connection params. */

//sec config
#define SEC_PARAM_IO_CAPABILITIES           GAP_IO_CAP_DISPLAY_ONLY                     /**< No I/O capabilities. (@enum gap_io_cap) */
#define SEC_PARAM_OOB                       0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_KEY_SIZE                  16                                          /**< Minimum encryption key size. 7 to 16 */
#define SEC_PARAM_BOND                      1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                      1                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                  0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IKEY                      GAP_KDIST_NONE                              /**< Initiator Key Distribution. (@enum gap_kdist) */
#define SEC_PARAM_RKEY                      GAP_KDIST_ENCKEY                            /**< Responder Key Distribution. (@enum gap_kdist) */
#define SEC_PARAM_SEC_MODE_LEVEL            GAP_NO_SEC                                  /**< Device security requirements (minimum security level). (@enum see gap_sec_req) */

//bond conifg
#define MAX_BOND_PEER                       5
#define BOND_STORE_ENABLE                   1
#define BOND_DATA_BASE_ADDR                 0x0103B000

/* profiles config  */
#define CFG_APP_DIS     1
#define CFG_PRF_DISS    1

#define CFG_APP_BATT    1
#define CFG_PRF_BASS    1 

#define CFG_PRF_HOGPD   1
#define CFG_APP_HID     1

// enable this patch if your MTK phone pair fail
#define _PATCH_ENC_RESPONDSE_ 1 


/* User config  */

#define NS_LOG_ERROR_ENABLE      1
#define NS_LOG_WARNING_ENABLE    1
#define NS_LOG_INFO_ENABLE       1
#define NS_LOG_DEBUG_ENABLE      0

#define NS_LOG_USART_ENABLE      1


#define NS_TIMER_ENABLE          1

#define FIRMWARE_VERSION         "1.0.0"
#define HARDWARE_VERSION         "1.0.0"



#endif // _APP_USER_CONFIG_H_

