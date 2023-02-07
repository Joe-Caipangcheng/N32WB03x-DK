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
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef _APP_USER_CONFIG_H_
#define _APP_USER_CONFIG_H_

/* Device name */
#define CUSTOM_DEVICE_NAME                  "NS_RDTS_CLIENT"                            /**<The device name of this device*/

/* connection config  */
#define MIN_CONN_INTERVAL                   15                                          /**< Minimum connection interval (15 ms) */
#define MAX_CONN_INTERVAL                   30                                          /**< Maximum connection interval (30 ms). */
#define SLAVE_LATENCY                       0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    5000                                        /**< Connection supervisory timeout (5000ms). */
    
#define FIRST_CONN_PARAMS_UPDATE_DELAY      (5000)                                      /**<  Time of initiating event to update connection params (5 seconds). */
    
/* sec config */    
#define SEC_PARAM_IO_CAPABILITIES           GAP_IO_CAP_NO_INPUT_NO_OUTPUT               /**< No I/O capabilities. (@enum gap_io_cap) */
#define SEC_PARAM_OOB                       0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_KEY_SIZE                  16                                          /**< Minimum encryption key size. 7 to 16 */
#define SEC_PARAM_BOND                      1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                      1                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                  0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IKEY                      GAP_KDIST_NONE                              /**< Initiator Key Distribution. (@enum gap_kdist) */
#define SEC_PARAM_RKEY                      GAP_KDIST_ENCKEY                            /**< Responder Key Distribution. (@enum gap_kdist) */
#define SEC_PARAM_SEC_MODE_LEVEL            GAP_NO_SEC                                  /**< Device security requirements (minimum security level). (@enum see gap_sec_req) */

/* bond conifg */
#define MAX_BOND_PEER                       5
#define BOND_STORE_ENABLE                   0
#define BOND_DATA_BASE_ADDR                 0x0103B000

/* profiles config  */
#define CFG_PRF_RDTSC                       1

/* scan config */
#define SCAN_PARAM_TYPE                     GAPM_SCAN_TYPE_OBSERVER                     /**< scan type (@enum gapm_scan_type) */ 
#define SCAN_PARAM_PROP_ACTIVE              1                                           /**< if requre scan respond data. 1:active, 0:passive*/ 
#define SCAN_PARAM_DUP_FILT_POL             GAPM_DUP_FILT_DIS                           /**< (@enum gapm_dup_filter_pol) */ 
#define SCAN_PARAM_INTV                     160                                         /**< scan interval, unit:0.625ms */ 
#define SCAN_PARAM_WD                       150                                         /**< scan window, unit:0.625ms */ 
#define SCAN_PARAM_DURATION                 0                                           /**< scan duration, unit:10ms, 0 mean forever */ 
#define SCAN_PARAM_CONNECT_EN               true 

/* User config  */
#define NS_LOG_ERROR_ENABLE                 1
#define NS_LOG_WARNING_ENABLE               1
#define NS_LOG_INFO_ENABLE                  1
#define NS_LOG_DEBUG_ENABLE                 0

#define NS_LOG_LPUART_ENABLE                1
#define NS_TIMER_ENABLE                     1

#define FIRMWARE_VERSION                    "1.0.0"
#define HARDWARE_VERSION                    "1.0.0"

#endif // _APP_USER_CONFIG_H_

