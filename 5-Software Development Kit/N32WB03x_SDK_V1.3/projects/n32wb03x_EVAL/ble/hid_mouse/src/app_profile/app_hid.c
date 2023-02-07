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
 * @file app_hid.c
 * @author Nations Firmware Team
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */


/**
 * @addtogroup APP
 * @{
 */

#include "rwip_config.h"            // SW configuration

#include <stdio.h>
#include <string.h>

#if (BLE_APP_HID)

/* Includes ------------------------------------------------------------------*/
#include "ns_ble.h"                    // Application Definitions
#include "ns_sec.h"                // Application Security Module API
#include "ns_ble_task.h"               // Application task definitions
#include "app_hid.h"                // HID Application Module Definitions
#include "hogpd_task.h"             // HID Over GATT Profile Device Role Functions
#include "prf_types.h"              // Profile common types Definition
#include "arch.h"                    // Platform Definitions
#include "prf.h"
#include "ke_timer.h"

#if (NVDS_SUPPORT)
#include "nvds.h"                   // NVDS Definitions
#endif //(NVDS_SUPPORT)
#include "hogpd.h"
#include "co_utils.h"               // Common functions

#if (KE_PROFILING)
#include "ke_mem.h"
#endif //(KE_PROFILING)
#include "app_gpio.h"
#include "app_ble.h" 
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/// Length of the HID  Report
#define APP_HID_CONSUMER_REPORT_LEN    (1)
#define APP_HID_MOUSE_REPORT_LEN       (6)
/// Length of the Report Descriptor for an HID Mouse
#define APP_HID_MOUSE_REPORT_MAP_LEN   (sizeof(app_hid_mouse_report_map))

/// Duration before connection update procedure if no report received (mouse is silent) - 20s
#define APP_HID_SILENCE_DURATION_1    0//(2000)
/// Duration before disconnection if no report is received after connection update - 60s
#define APP_HID_SILENCE_DURATION_2     (6000)

/// Number of reports that can be sent
#define APP_HID_NB_SEND_REPORT         (10)

/// States of the Application HID Module
enum app_hid_states
{
    /// Module is disabled (Service not added in DB)
    APP_HID_DISABLED,
    /// Module is idle (Service added but profile not enabled)
    APP_HID_IDLE,
    /// Module is enabled (Device is connected and the profile is enabled)
    APP_HID_ENABLED,
    /// The application can send reports
    APP_HID_READY,
    /// Waiting for a report
    APP_HID_WAIT_REP,

    APP_HID_STATE_MAX,
};

/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/// HID Application Module Environment Structure
static struct app_hid_env_tag app_hid_env;

/// HID Mouse Report Descriptor
static const uint8_t app_hid_mouse_report_map[] =
{
    /**
     *  --------------------------------------------------------------------------
     *  Bit      |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
     *  --------------------------------------------------------------------------
     *  Byte 0   |               Not Used                | Middle| Right | Left  |
     *  --------------------------------------------------------------------------
     *  Byte 1   |                     X Axis Relative Movement                  |
     *  --------------------------------------------------------------------------
     *  Byte 2   |                     Y Axis Relative Movement                  |
     *  --------------------------------------------------------------------------
     *  Byte 3   |                     Wheel Relative Movement                   |
     *  --------------------------------------------------------------------------
     */

    0x05, 0x01,     /// USAGE PAGE (Generic Desktop)
    0x09, 0x02,     /// USAGE (Mouse)
    
    0xA1, 0x01,     /// COLLECTION (Application)
    
    0x85, 0x01,     /// REPORT ID (1) - MANDATORY
    0x09, 0x01,     ///     USAGE (Pointer)
    0xA1, 0x00,     ///     COLLECTION (Physical)

    /**
     * ----------------------------------------------------------------------------
     * BUTTONS
     * ----------------------------------------------------------------------------
     */
    0x05, 0x09,     ///         USAGE PAGE (Buttons)
    0x19, 0x01,     ///         USAGE MINIMUM (1)
    0x29, 0x08,     ///         USAGE MAXIMUM (8)
    0x15, 0x00,     ///         LOGICAL MINIMUM (0)
    0x25, 0x01,     ///         LOGICAL MAXIMUM (1)
    0x75, 0x01,     ///         REPORT SIZE (1)
    0x95, 0x08,     ///         REPORT COUNT (8)
    0x81, 0x02,     ///         INPUT (Data, Variable, Absolute)

    /**
     * ----------------------------------------------------------------------------
     * MOVEMENT DATA
     * ----------------------------------------------------------------------------
     */
    0x05, 0x01,     ///         USAGE PAGE (Generic Desktop)
    0x16, 0x08, 0xFF, ///       LOGICAL MINIMUM (-255)
    0x26, 0xFF, 0x00, ///       LOGICAL MAXIMUM (255)
    0x75, 0x10,     ///         REPORT SIZE (16)
    0x95, 0x02,     ///         REPORT COUNT (2)
    0x09, 0x30,     ///         USAGE (X)
    0x09, 0x31,     ///         USAGE (Y)
    0x81, 0x06,     ///         INPUT (Data, Variable, Relative)

    0x15, 0x81,     ///         LOGICAL MINIMUM (-127)
    0x25, 0x7F,     ///         LOGICAL MAXIMUM (127)
    0x75, 0x08,     ///         REPORT SIZE (8)
    0x95, 0x01,     ///         REPORT COUNT (1)
    0x09, 0x38,     ///         USAGE (Wheel)
    0x81, 0x06,     ///         INPUT (Data, Variable, Relative)

    0xC0,           ///     END COLLECTION (Physical)
    0xC0,            /// END COLLECTION (Application)

    // Report ID 2: Advanced buttons
    0x05, 0x0C,       // Usage Page (Consumer)
    0x09, 0x01,       // Usage (Consumer Control)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x02,       // Report Id (2)
    0x15, 0x00,       // Logical minimum (0)
    0x25, 0x01,       // Logical maximum (1)
    0x75, 0x01,       // Report Size (1)
    0x95, 0x08,       // Report Count (8)

    0x09, 0xCD,       // Usage (Play/Pause)        //test, for audio final    
    0x09, 0xB6,       // Usage (Scan Previous Track)
    0x09, 0xB5,       // Usage (Scan Next Track)
    0x09, 0xEA,       // Usage (Volume Down)
    0x09, 0xE9,       // Usage (Volume Up)
    0x09, 0xE2,       // Usage (Mute)
    0x09, 0xCC,       // Usage (Stop eject)    
    0x09, 0xB7,       // Usage (Stop) 
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
  
    0xC0,              // End Collection

};



/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


void app_hid_init(void)
{
    NS_LOG_DEBUG("%s\r\n",__func__);
    
    // Reset the environment
    memset(&app_hid_env, 0, sizeof(app_hid_env));

    app_hid_env.nb_report = APP_HID_NB_SEND_REPORT;


    app_hid_env.timeout = APP_HID_SILENCE_DURATION_1;
    //register application subtask to app task
    struct prf_task_t prf;
    prf.prf_task_id = TASK_ID_HOGPD;
    prf.prf_task_handler = &app_hid_handlers;
    ns_ble_prf_task_register(&prf);
    
    //register get itf function to prf.c
    struct prf_get_func_t get_func;
    get_func.task_id = TASK_ID_HOGPD;
    get_func.prf_itf_get_func = hogpd_prf_itf_get;
    prf_get_itf_func_register(&get_func);
    
}


void app_hid_add_hids(void)
{
    struct hogpd_db_cfg *db_cfg;
    // Prepare the HOGPD_CREATE_DB_REQ message
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
                                                   TASK_GAPM, TASK_APP,
                                                   gapm_profile_task_add_cmd, sizeof(struct hogpd_db_cfg));

    NS_LOG_DEBUG("%s\r\n",__func__);
    // Fill message
    req->operation   = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl     = PERM(SVC_AUTH, UNAUTH); // NO_AUTH UNAUTH AUTH
    req->prf_task_id = TASK_ID_HOGPD;
    req->app_task    = TASK_APP;
    req->start_hdl   = 0;

    // Set parameters
    db_cfg = (struct hogpd_db_cfg* ) req->param;

    // Only one HIDS instance is useful
    db_cfg->hids_nb = 1;

    // The device is a mouse
    db_cfg->cfg[0].svc_features = HOGPD_CFG_MOUSE;//HOGPD_CFG_MOUSE_BIT;

    // Only one Report Characteristic is requested
    db_cfg->cfg[0].report_nb    = 2; 

    db_cfg->cfg[0].report_id[0] = 1;
    db_cfg->cfg[0].report_char_cfg[0] = HOGPD_CFG_REPORT_IN;
    
    db_cfg->cfg[0].report_id[1] = 2;
    db_cfg->cfg[0].report_char_cfg[1] = HOGPD_CFG_REPORT_IN;

    // HID Information
    db_cfg->cfg[0].hid_info.bcdHID       = 0x0111;         // HID Version 1.11
    db_cfg->cfg[0].hid_info.bCountryCode = 0x00;
    db_cfg->cfg[0].hid_info.flags        = HIDS_REMOTE_WAKE_CAPABLE | HIDS_NORM_CONNECTABLE;

    // Send the message
    ke_msg_send(req);
    
    app_hid_init();
}




/*
 * @brief Function called when get connection complete event from the GAP
 *
 */
void app_hid_enable_prf(uint8_t conidx)
{
    uint16_t ntf_cfg;
    NS_LOG_DEBUG("%s,idx %x\r\n",__func__,conidx);

    // Store the connection handle
    app_hid_env.conidx = conidx;

    // Allocate the message
    struct hogpd_enable_req * req = KE_MSG_ALLOC(HOGPD_ENABLE_REQ,
                                                 prf_get_task_from_id(TASK_ID_HOGPD),
                                                 TASK_APP,
                                                 hogpd_enable_req);

    // Fill in the parameter structure
    req->conidx     = conidx;
    // Notifications are disabled
    ntf_cfg         = 0;

    // Go to Enabled state
    app_hid_env.state = APP_HID_ENABLED;

    // If first connection with the peer device
    if (ns_sec_get_bond_status())
    {
        app_hid_env.state = APP_HID_READY;
        app_hid_env.nb_report = APP_HID_NB_SEND_REPORT;
        ntf_cfg = 0xC2;
        NS_LOG_DEBUG("HID ready for bonded device\r\n");

    }
 
    req->ntf_cfg[conidx] = ntf_cfg;

    // Send the message
    ke_msg_send(req);
}







/*
 * @brief Function called from PS2 driver
 *
 */
void app_hid_send_mouse_report(struct ps2_mouse_msg report)
{

    NS_LOG_DEBUG("HID report, state:%d, x:%d, y:%d \r\n",app_hid_env.state, report.x, report.y);
    switch (app_hid_env.state)
    {
        case (APP_HID_READY):
        {
            // Check if the report can be sent
            if (app_hid_env.nb_report)
            {
                // Buffer used to create the Report
                uint8_t report_buff[APP_HID_MOUSE_REPORT_LEN];
                // X, Y and wheel relative movements
                int16_t x;
                int16_t y;

                // Clean the report buffer
                memset(&report_buff[0], 0, APP_HID_MOUSE_REPORT_LEN);

                // Set the button states
                report_buff[0] = (report.b & 0x07);

                // If X value is negative
                if (report.b & 0x10)
                {
                    report.x = ~report.x;
                    report.x += 1;
                    x = (int16_t)report.x;
                    x *= (-1);
                }
                else
                {
                    x = (int16_t)report.x;
                }

                // If Y value is negative
                if (report.b & 0x20)
                {
                    report.y = ~report.y;
                    report.y += 1;
                    y = (int16_t)report.y;
                }
                else
                {
                    y = (int16_t)report.y;
                    y *= (-1);
                }


                // Set the X and Y movement value in the report
                co_write16p(&report_buff[1], x);
                co_write16p(&report_buff[3], y);
                report_buff[5] =(signed char) (-1) * report.w;

                // Allocate the HOGPD_REPORT_UPD_REQ message
                struct hogpd_report_upd_req * req = KE_MSG_ALLOC_DYN(HOGPD_REPORT_UPD_REQ,
                                                                  prf_get_task_from_id(TASK_ID_HOGPD),
                                                                  TASK_APP,
                                                                  hogpd_report_upd_req,
                                                                  APP_HID_MOUSE_REPORT_LEN);

                req->conidx  = app_hid_env.conidx;
                //now fill report
                req->report.hid_idx  = app_hid_env.conidx;
                req->report.type     = HOGPD_REPORT; //HOGPD_BOOT_MOUSE_INPUT_REPORT;//
                req->report.idx      = 0; //0 for boot reports and report map
                req->report.length   = APP_HID_MOUSE_REPORT_LEN;
                memcpy(&req->report.value[0], &report_buff[0], APP_HID_MOUSE_REPORT_LEN);

                ke_msg_send(req);

                app_hid_env.nb_report--;

                // Restart the mouse timeout timer if needed
                if (app_hid_env.timeout != 0)
                {
                    ke_timer_set(APP_HID_MOUSE_TIMEOUT_TIMER, TASK_APP, (uint16_t)(app_hid_env.timeout));
                    app_hid_env.timer_enabled = true;
                }
            }
        } break;

        case (APP_HID_WAIT_REP):
        {
            // Requested connection parameters
            struct gapc_conn_param conn_param;

            /*
             * Requested connection interval: 10ms
             * Latency: 25
             * Supervision Timeout: 2s
             */
            conn_param.intv_min = 8;
            conn_param.intv_max = 8;
            conn_param.latency  = 25;
            conn_param.time_out = 200;

            ns_ble_update_param(&conn_param);
            // Restart the mouse timeout timer if needed
            if (app_hid_env.timeout != 0)
            {
                ke_timer_set(APP_HID_MOUSE_TIMEOUT_TIMER, TASK_APP, (uint16_t)(app_hid_env.timeout));
                app_hid_env.timer_enabled = true;
            }

            // Go back to the ready state
            app_hid_env.state = APP_HID_READY;
        } break;

        case (APP_HID_IDLE):
        {
            // Try to restart advertising if needed
//            app_update_adv_state(true);
        } break;
        
                
        // DISABLE and ENABLED states
        default:
        {
            // Drop the message
        } break;
    }

}


/*
 * @brief Function  
 *
 */
void app_hid_send_consumer_report(uint8_t* report)
{
    NS_LOG_DEBUG("Consumer,state:%d ,%d\r\n",app_hid_env.state,app_hid_env.nb_report);
    switch (app_hid_env.state)
    {
        case (APP_HID_READY):
        {
            // Check if the report can be sent
            if (app_hid_env.nb_report)
            {
                // Allocate the HOGPD_REPORT_UPD_REQ message
                struct hogpd_report_upd_req * req = KE_MSG_ALLOC_DYN(HOGPD_REPORT_UPD_REQ,
                                                                  prf_get_task_from_id(TASK_ID_HOGPD),
                                                                  TASK_APP,
                                                                  hogpd_report_upd_req,
                                                                  APP_HID_CONSUMER_REPORT_LEN);

                req->conidx  = app_hid_env.conidx;
                //now fill report
                req->report.hid_idx  = app_hid_env.conidx;
                req->report.type     = HOGPD_REPORT; //HOGPD_BOOT_MOUSE_INPUT_REPORT;//
                req->report.idx      = 1; //repoort id 2 report map
                req->report.length   = APP_HID_CONSUMER_REPORT_LEN;
                memcpy(&req->report.value[0], &report[0],APP_HID_CONSUMER_REPORT_LEN);

                ke_msg_send(req);

                app_hid_env.nb_report--;

                // Restart the mouse timeout timer if needed
                if (app_hid_env.timeout != 0)
                {
                    ke_timer_set(APP_HID_MOUSE_TIMEOUT_TIMER, TASK_APP, (uint16_t)(app_hid_env.timeout));
                    app_hid_env.timer_enabled = true;
                }
            }
        } break;

        case (APP_HID_WAIT_REP):
        {
            // Requested connection parameters
            struct gapc_conn_param conn_param;

            /*
             * Requested connection interval: 10ms
             * Latency: 25
             * Supervision Timeout: 2s
             */
            conn_param.intv_min = 8;
            conn_param.intv_max = 8;
            conn_param.latency  = 25;
            conn_param.time_out = 200;
            ns_ble_update_param(&conn_param);

            // Restart the mouse timeout timer if needed
            if (app_hid_env.timeout != 0)
            {
                ke_timer_set(APP_HID_MOUSE_TIMEOUT_TIMER, TASK_APP, (uint16_t)(app_hid_env.timeout));
                app_hid_env.timer_enabled = true;
            }

            // Go back to the ready state
            app_hid_env.state = APP_HID_READY;
        } break;

        case (APP_HID_IDLE):
        {
            // Try to restart advertising if needed
//            app_update_adv_state(true);
        } break;
        
                
        // DISABLE and ENABLED states
        default:
        {
            // Drop the message
        } break;
    }
#if (KE_PROFILING)
//    app_display_hdl_env_size(0xFFFF, ke_get_mem_usage(KE_MEM_ENV));
//    app_display_hdl_db_size(0xFFFF, ke_get_mem_usage(KE_MEM_ATT_DB));
//    app_display_hdl_msg_size((uint16_t)ke_get_max_mem_usage(), ke_get_mem_usage(KE_MEM_KE_MSG));
    #endif //(KE_PROFILING)
}


bool is_app_hid_ready(void)
{
    if (app_hid_env.state == APP_HID_READY)
    {
        return true;
    }

    return false;
}


/*
 * MESSAGE HANDLERS
 */


static int hogpd_ctnl_pt_ind_handler(ke_msg_id_t const msgid,
                                     struct hogpd_ctnl_pt_ind const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
   NS_LOG_DEBUG("%s\r\n",__func__);

    if (param->conidx == app_hid_env.conidx)
    {
        //make use of param->hid_ctnl_pt
        struct hogpd_report_cfm *req = KE_MSG_ALLOC_DYN(HOGPD_REPORT_CFM,
                                                        prf_get_task_from_id(TASK_ID_HOGPD),/* src_id */
                                                        TASK_APP,
                                                        hogpd_report_cfm,
                                                        0);

        req->conidx = param->conidx; ///app_hid_env.conidx; ///???
        /// Operation requested (read/write @see enum hogpd_op)
        req->operation = HOGPD_OP_REPORT_WRITE;
        /// Status of the request
        req->status = GAP_ERR_NO_ERROR;  ///???
        /// Report Info
        //req->report;
        /// HIDS Instance
        req->report.hid_idx = app_hid_env.conidx; ///???
        /// type of report (@see enum hogpd_report_type)
        req->report.type = (uint8_t)-1;//outside 
        /// Report Length (uint8_t)
        req->report.length = 0;
        /// Report Instance - 0 for boot reports and report map
        req->report.idx = 0;
        /// Report data
        

        // Send the message
        ke_msg_send(req);
    }
    return (KE_MSG_CONSUMED);
}




static int hogpd_ntf_cfg_ind_handler(ke_msg_id_t const msgid,
                                     struct hogpd_ntf_cfg_ind const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    NS_LOG_DEBUG("%s,v_idx;%x,p_idx;%x\r\n",__func__,app_hid_env.conidx,param->conidx);
    if (app_hid_env.conidx == param->conidx)
    {
        if ((param->ntf_cfg[param->conidx] & HOGPD_CFG_REPORT_NTF_EN ) != 0)
        {
            // The device is ready to send reports to the peer device
            app_hid_env.state = APP_HID_READY;
            NS_LOG_INFO("HID Ready\r\n");
        }
        else
        {
            // Come back to the Enabled state
            if (app_hid_env.state == APP_HID_READY)
            {
                app_hid_env.state = APP_HID_ENABLED;
            }
            NS_LOG_DEBUG("HID enable\r\n");
        }
        NS_LOG_DEBUG("ntf_cfg:0x%x\r\n",param->ntf_cfg[param->conidx]);    
        #if (NVDS_SUPPORT)
        // Store the notification configuration in the database
        if (nvds_put(NVDS_TAG_MOUSE_NTF_CFG, NVDS_LEN_MOUSE_NTF_CFG,
                     (uint8_t *)&param->ntf_cfg[param->conidx]) != NVDS_OK)
        {
            // Should not happen
            ASSERT_ERR(0);
        }
        #endif //(NVDS_SUPPORT)
    }

    return (KE_MSG_CONSUMED);
}

static int hogpd_report_req_ind_handler(ke_msg_id_t const msgid,
                                    struct hogpd_report_req_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    NS_LOG_DEBUG("%s\r\n",__func__);
    if ((param->operation == HOGPD_OP_REPORT_READ) && (param->report.type == HOGPD_REPORT_MAP))
    {
        struct hogpd_report_cfm *req = KE_MSG_ALLOC_DYN(HOGPD_REPORT_CFM,
                                                        src_id, 
                                                        dest_id, 
                                                        hogpd_report_cfm,
                                                        APP_HID_MOUSE_REPORT_MAP_LEN);

        req->conidx = app_hid_env.conidx; ///???
        /// Operation requested (read/write @see enum hogpd_op)
        req->operation = HOGPD_OP_REPORT_READ;
        /// Status of the request
        req->status = GAP_ERR_NO_ERROR;  ///???
        /// Report Info
        //req->report;
        /// HIDS Instance
        req->report.hid_idx = param->report.hid_idx;///   ???///app_hid_env.conidx; ///???
        /// type of report (@see enum hogpd_report_type)
        req->report.type = HOGPD_REPORT_MAP;
        /// Report Length (uint8_t)
        req->report.length = APP_HID_MOUSE_REPORT_MAP_LEN;
        /// Report Instance - 0 for boot reports and report map
        req->report.idx = 0;
        /// Report data
         memcpy(&req->report.value[0], &app_hid_mouse_report_map[0], APP_HID_MOUSE_REPORT_MAP_LEN);

        // Send the message
        ke_msg_send(req);
    }
    else
    {
        if (param->report.type == HOGPD_BOOT_MOUSE_INPUT_REPORT)
        { //request of boot mouse report
            struct hogpd_report_cfm *req = KE_MSG_ALLOC_DYN(HOGPD_REPORT_CFM,
                                                            prf_get_task_from_id(TASK_ID_HOGPD),/* src_id */
                                                            TASK_APP,
                                                            hogpd_report_cfm,
                                                            0/*param->report.length*/);

            req->conidx = param->conidx; ///app_hid_env.conidx; ///???
            /// Operation requested (read/write @see enum hogpd_op)
            req->operation = HOGPD_OP_REPORT_READ;
            /// Status of the request
            req->status = GAP_ERR_NO_ERROR;  ///???
            /// HIDS Instance
            req->report.hid_idx = app_hid_env.conidx; ///???
            /// type of report (@see enum hogpd_report_type)
            req->report.type = param->report.type;//-1;//outside 
            /// Report Length (uint8_t)
            req->report.length = 0; //param->report.length;
            /// Report Instance - 0 for boot reports and report map
            req->report.idx = param->report.idx; //0;
            /// Report data

            // Send the message
            ke_msg_send(req);
        }
        else
        if (param->report.type == HOGPD_REPORT)
        { //request of mouse report
            struct hogpd_report_cfm *req = KE_MSG_ALLOC_DYN(HOGPD_REPORT_CFM,
                                                            prf_get_task_from_id(TASK_ID_HOGPD),/* src_id */
                                                            TASK_APP,
                                                            hogpd_report_cfm,
                                                            8/*param->report.length*/);

            req->conidx = param->conidx; ///app_hid_env.conidx; ///???
            /// Operation requested (read/write @see enum hogpd_op)
            req->operation = HOGPD_OP_REPORT_READ;
            /// Status of the request
            req->status = GAP_ERR_NO_ERROR;  ///???
            /// Report Info
            //req->report;
            /// HIDS Instance
            req->report.hid_idx = app_hid_env.conidx; ///???
            /// type of report (@see enum hogpd_report_type)
            req->report.type = param->report.type;//-1;//outside 
            /// Report Length (uint8_t)
            req->report.length = 8; //param->report.length;
            /// Report Instance - 0 for boot reports and report map
            req->report.idx = param->report.idx; //0;
            /// Report data
            memset(&req->report.value[0], 0, 8); //???
            req->report.value[0] = param->report.hid_idx;    /// HIDS Instance
            req->report.value[1] = param->report.type;    /// type of report (@see enum hogpd_report_type)
            req->report.value[2] = param->report.length;    /// Report Length (uint8_t)
            req->report.value[3] = param->report.idx;    /// Report Instance - 0 for boot reports and report map

            // Send the message
            ke_msg_send(req);
        }
        else
        {
            struct hogpd_report_cfm *req = KE_MSG_ALLOC_DYN(HOGPD_REPORT_CFM,
                                                            prf_get_task_from_id(TASK_ID_HOGPD),/* src_id */
                                                            TASK_APP,
                                                            hogpd_report_cfm,
                                                            8/*param->report.length*/);

            req->conidx = param->conidx; ///app_hid_env.conidx; ///???
            /// Operation requested (read/write @see enum hogpd_op)
            req->operation = HOGPD_OP_REPORT_READ;
            /// Status of the request
            req->status = GAP_ERR_NO_ERROR;  ///???
            /// Report Info
            //req->report;
            /// HIDS Instance
            req->report.hid_idx = app_hid_env.conidx; ///???
            /// type of report (@see enum hogpd_report_type)
            req->report.type = param->report.type;//-1;//outside 
            /// Report Length (uint8_t)
            req->report.length = 8; //param->report.length;
            /// Report Instance - 0 for boot reports and report map
            req->report.idx = param->report.idx; //0;
            /// Report data
            memset(&req->report.value[0], 0, 8); //???
            req->report.value[0] = param->report.hid_idx;    /// HIDS Instance
            req->report.value[1] = param->report.type;    /// type of report (@see enum hogpd_report_type)
            req->report.value[2] = param->report.length;    /// Report Length (uint8_t)
            req->report.value[3] = param->report.idx;    /// Report Instance - 0 for boot reports and report map

            // Send the message
            ke_msg_send(req);
        }
    }

    return (KE_MSG_CONSUMED);
}

static int hogpd_proto_mode_req_ind_handler(ke_msg_id_t const msgid,
                                        struct hogpd_proto_mode_req_ind const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    NS_LOG_DEBUG("%s\r\n",__func__);
    if ((param->conidx == app_hid_env.conidx) && (param->operation == HOGPD_OP_PROT_UPDATE))
    {

        //make use of param->proto_mode
        struct hogpd_proto_mode_cfm *req = KE_MSG_ALLOC_DYN(HOGPD_PROTO_MODE_CFM,
                                                        prf_get_task_from_id(TASK_ID_HOGPD),/* src_id */
                                                        TASK_APP,
                                                        hogpd_proto_mode_cfm,
                                                        0);
        /// Connection Index
        req->conidx = app_hid_env.conidx; 
        /// Status of the request
        req->status = GAP_ERR_NO_ERROR;
        /// HIDS Instance
        req->hid_idx = app_hid_env.conidx;
        /// New Protocol Mode Characteristic Value
        req->proto_mode = param->proto_mode;
        

        // Send the message
        ke_msg_send(req);
    }
    else
    {
        struct hogpd_proto_mode_cfm *req = KE_MSG_ALLOC_DYN(HOGPD_PROTO_MODE_CFM,
                                                        prf_get_task_from_id(TASK_ID_HOGPD),/* src_id */
                                                        TASK_APP,
                                                        hogpd_proto_mode_cfm,
                                                        0);
        /// Status of the request
        req->status = ATT_ERR_APP_ERROR;

        /// Connection Index
        req->conidx = app_hid_env.conidx;
        /// HIDS Instance
        req->hid_idx = app_hid_env.conidx;
        /// New Protocol Mode Characteristic Value
        req->proto_mode = param->proto_mode;
        
        // Send the message
        ke_msg_send(req);
    }
    return (KE_MSG_CONSUMED);
}


static int hogpd_report_upd_handler(ke_msg_id_t const msgid,
                                   struct hogpd_report_upd_rsp const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    NS_LOG_DEBUG("%s,status:%x \r\n",__func__,param->status);
    if (app_hid_env.conidx == param->conidx)
    {
        if (GAP_ERR_NO_ERROR == param->status)
        {
            if (app_hid_env.nb_report < APP_HID_NB_SEND_REPORT)
            {
                app_hid_env.nb_report++;
            }


        }
        else
        {
            // we get this message if error occur while sending report
            // most likely - disconnect
            // Go back to the ready state
            app_hid_env.state = APP_HID_IDLE;
            // change mode
            // restart adv
            // Try to restart advertising if needed
//            app_update_adv_state(true);

            //report was not success - need to restart???
        }
    }
    return (KE_MSG_CONSUMED);
}

static int hogpd_enable_rsp_handler(ke_msg_id_t const msgid,
                                     struct hogpd_enable_rsp const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    NS_LOG_DEBUG("%s,idx %x,status %x\r\n",__func__,param->conidx,param->status);
    return (KE_MSG_CONSUMED);
}

/**
 * @brief Function called when the APP_HID_MOUSE_TIMEOUT_TIMER expires.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 */
int app_hid_mouse_timeout_timer_handler(ke_msg_id_t const msgid,void const *param)
{
    app_hid_env.timer_enabled = false;
    NS_LOG_DEBUG("%s\r\n",__func__);
    if (app_hid_env.state == APP_HID_READY)
    {
        // Requested connection parameters
        struct gapc_conn_param conn_param;
        // Timer value
        uint16_t timer_val;

        /*
         * Request an update of the connection parameters
         * Requested connection interval: 10ms
         * Latency: 200
         * Supervision Timeout: 5s
         */
        conn_param.intv_min = 8;
        conn_param.intv_max = 8;
        conn_param.latency  = 200;
        conn_param.time_out = 500;
        ns_ble_update_param(&conn_param);

        // Go to the Wait for Report state
        app_hid_env.state = APP_HID_WAIT_REP;

        timer_val = APP_HID_SILENCE_DURATION_2;

        // Relaunch the timer
        ke_timer_set(APP_HID_MOUSE_TIMEOUT_TIMER, TASK_APP, timer_val);
        app_hid_env.timer_enabled = true;
    }
    else if (app_hid_env.state == APP_HID_WAIT_REP)
    {
      // Disconnect the link with the device
        ns_ble_disconnect();


        // Go back to the ready state
        app_hid_env.state = APP_HID_IDLE;
    }

    return (KE_MSG_CONSUMED);
}





/**
 * @brief
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 */
static int app_hid_msg_dflt_handler(ke_msg_id_t const msgid,
                                    void const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    // Drop the message
    NS_LOG_DEBUG("%s\r\n",__func__);
    return (KE_MSG_CONSUMED);
}

/**
 * @brief Set the value of the Report Map Characteristic in the database
 */
void app_hid_set_report_map(void);

/*
 * LOCAL VARIABLE DEFINITIONS
 */

/// Default State handlers definition
const struct ke_msg_handler app_hid_msg_handler_list[] =
{
    // Note: first message is latest message checked by kernel so default is put on top.
    {KE_MSG_DEFAULT_HANDLER,        (ke_msg_func_t)app_hid_msg_dflt_handler},

    {HOGPD_ENABLE_RSP,              (ke_msg_func_t)hogpd_enable_rsp_handler},
    {HOGPD_NTF_CFG_IND,             (ke_msg_func_t)hogpd_ntf_cfg_ind_handler},
    {HOGPD_REPORT_REQ_IND,          (ke_msg_func_t)hogpd_report_req_ind_handler},
    {HOGPD_PROTO_MODE_REQ_IND,      (ke_msg_func_t)hogpd_proto_mode_req_ind_handler},
    {HOGPD_CTNL_PT_IND,             (ke_msg_func_t)hogpd_ctnl_pt_ind_handler},
    {HOGPD_REPORT_UPD_RSP,          (ke_msg_func_t)hogpd_report_upd_handler},

};

const struct app_subtask_handlers app_hid_handlers = APP_HANDLERS(app_hid);

#endif //(BLE_APP_HID)

/// @} APP
