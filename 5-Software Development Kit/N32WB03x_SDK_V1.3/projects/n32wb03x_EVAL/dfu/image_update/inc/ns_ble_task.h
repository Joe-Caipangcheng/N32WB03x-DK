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
 * @file ns_ble_task.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */


#ifndef APP_TASK_H_
#define APP_TASK_H_

/**
 * @addtogroup APPTASK Task
 * @ingroup APP
 * @brief Routes ALL messages to/from APP block.
 *
 * The APPTASK is the block responsible for bridging the final application with the
 * RWBLE software host stack. It communicates with the different modules of the BLE host,
 * i.e. @ref SMP, @ref GAP and @ref GATT.
 *
 * @{
 **/

/* Includes ------------------------------------------------------------------*/

#include "rwip_config.h"             // SW configuration

#if (BLE_APP_PRESENT)

#include <stdint.h>         // Standard Integer
#include "rwip_task.h"      // Task definitions
#include "ke_task.h"        // Kernel Task

/* Define ------------------------------------------------------------*/

/// Number of APP Task Instances
#define APP_IDX_MAX                 (1)



/* Typedef -----------------------------------------------------------*/
 
 /// Process event response
enum process_event_response
{
    /// Handled
    PR_EVENT_HANDLED = 0,

    /// Unhandled
    PR_EVENT_UNHANDLED
};


/// States of APP task
enum app_state
{
    /// Initialization state
    APP_INIT,
    /// Database create state
    APP_CREATE_DB,
    /// Ready State
    APP_READY,
    /// Connected state
    APP_CONNECTED,

    /// Number of defined states.
    APP_STATE_MAX
};


/// APP Task messages
/*@TRACE*/
enum app_msg_id
{
    APP_DUMMY_MSG = TASK_FIRST_MSG(TASK_ID_APP),

    #if (BLE_APP_PRF)
    #if (BLE_APP_HT)
    /// Timer used to refresh the temperature measurement value
    APP_HT_MEAS_INTV_TIMER,
    #endif //(BLE_APP_HT)

    #if (BLE_APP_HID)
    /// Timer used to disconnect the moue if no activity is detecter
    APP_HID_MOUSE_TIMEOUT_TIMER,
    #endif //(BLE_APP_HID)
    #endif //(BLE_APP_PRF)


    //user event
    APP_PARAMS_UPDATE_EVT,
    APP_CUSTS_TEST_EVT,
    APP_HID_DATA_PROCESS_EVT,
    APP_DFU_BLE_RESET_TIMER,

#if (NS_TIMER_ENABLE)
    /*ns timer*/
    APP_CANCEL_TIMER,
    APP_MODIFY_TIMER,
    //Do not alter the order of the next messages
    //they are considered a range
    NS_TIMER_API_MES0,
    NS_TIMER_API_MES1=NS_TIMER_API_MES0+1,
    NS_TIMER_API_MES2=NS_TIMER_API_MES0+2,
    NS_TIMER_API_MES3=NS_TIMER_API_MES0+3,
    NS_TIMER_API_MES4=NS_TIMER_API_MES0+4,
    NS_TIMER_API_MES5=NS_TIMER_API_MES0+5,
    NS_TIMER_API_MES6=NS_TIMER_API_MES0+6,
    NS_TIMER_API_MES7=NS_TIMER_API_MES0+7,
    NS_TIMER_API_MES8=NS_TIMER_API_MES0+8,
    NS_TIMER_API_MES9=NS_TIMER_API_MES0+9,
    NS_TIMER_API_LAST_MES=NS_TIMER_API_MES9,
#endif //NS_TIMER_ENABLE

};


/* Public variables ---------------------------------------------------------*/

/// @} APPTASK

#endif //(BLE_APP_PRESENT)

#endif // APP_TASK_H_
