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
 * @file rwip.c
 * @author Nations Firmware Team
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */


#include "rwip_config.h"     // RW SW configuration

#if (NVDS_SUPPORT)
#include "nvds.h"         // NVDS definitions
#endif // NVDS_SUPPORT

#if (BT_EMB_PRESENT)
#include "rwbt.h"            // rwbt definitions
#endif //BT_EMB_PRESENT

#if (BLE_EMB_PRESENT)
#include "rwble.h"           // rwble definitions
#endif //BLE_EMB_PRESENT

#if (BLE_HOST_PRESENT)
#include "rwble_hl.h"        // BLE HL definitions
#include "gapc.h"
#include "gapm.h"
#include "gattc.h"
#include "l2cc.h"
#endif //BLE_HOST_PRESENT


#if (BT_EMB_PRESENT)
#include "ld.h"
#endif //BT_EMB_PRESENT

#if (DISPLAY_SUPPORT)
#include "display.h"         // display definitions
#include "co_utils.h"        // toolbox
//#include "plf.h"             // platform definition
#if (BT_EMB_PRESENT)
#include "reg_btcore.h"
#endif // (BT_EMB_PRESENT)
#if (BLE_EMB_PRESENT)
#include "reg_blecore.h"
#endif // (BLE_EMB_PRESENT)
#if (BT_DUAL_MODE)
#include "reg_ipcore.h"
#endif // (BT_DUAL_MODE)
#endif //DISPLAY_SUPPORT

#if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
#include "sch_arb.h"            // Scheduling Arbiter
#include "sch_prog.h"           // Scheduling Programmer
#include "sch_plan.h"           // Scheduling Planner
#include "sch_slice.h"          // Scheduling Slicer
#include "sch_alarm.h"          // Scheduling Alarm
#endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)

#if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
#include "rf.h"              // RF definitions
#endif //BT_EMB_PRESENT || BLE_EMB_PRESENT

#if (H4TL_SUPPORT)
#include "h4tl.h"
#endif //H4TL_SUPPORT

#if (AHI_TL_SUPPORT)
#include "ahi.h"
#endif //AHI_TL_SUPPORT

#if (HCI_PRESENT)
#include "hci.h"             // HCI definition
#endif //HCI_PRESENT


#if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
#include "ecc_p256.h"        // ECC P256 library
#endif // (BT_EMB_PRESENT || BLE_EMB_PRESENT)

#include "aes.h"             // For AES functions

#if (BLE_EMB_PRESENT)
#include "reg_blecore.h"        // ble core registers
#endif //BLE_EMB_PRESENT

#if (BT_EMB_PRESENT)
#include "reg_btcore.h"         // bt core registers
#endif //BT_EMB_PRESENT

#include "dbg.h"             // debug definition
#include "global_func.h"

#define rwip_rst_state         REG32(0X20000000)

extern void llhwc_phy_prerx_flash(void);

void rwip_init(uint32_t error)
{
    // IP initialization
    rwip_rst_state = RWIP_INIT;

    // Initialize kernel
    ke_init();
    
    // Initialize memory heap used by kernel.
    // Memory allocated for environment variables
    //ke_mem_init(KE_MEM_ENV,           (uint8_t*)rwip_heap_env,     RWIP_CALC_HEAP_LEN_IN_BYTES(RWIP_HEAP_ENV_SIZE));
    ke_mem_init(KE_MEM_ENV,           (uint8_t*)g_rwip_heap_env,     g_rwip_heap_env_size);

    #if (BLE_HOST_PRESENT)
    // Memory allocated for Attribute database
    //ke_mem_init(KE_MEM_ATT_DB,        (uint8_t*)rwip_heap_db,      RWIP_CALC_HEAP_LEN_IN_BYTES(RWIP_HEAP_DB_SIZE));
    ke_mem_init(KE_MEM_ATT_DB,        (uint8_t*)g_rwip_heap_db,      g_rwip_heap_db_size);
 
    #endif // (BLE_HOST_PRESENT)
    // Memory allocated for kernel messages
    //ke_mem_init(KE_MEM_KE_MSG,        (uint8_t*)rwip_heap_msg,     RWIP_CALC_HEAP_LEN_IN_BYTES(RWIP_HEAP_MSG_SIZE));
    ke_mem_init(KE_MEM_KE_MSG,        (uint8_t*)g_rwip_heap_msg,     g_rwip_heap_msg_size);

    // Non Retention memory block
    //ke_mem_init(KE_MEM_NON_RETENTION, (uint8_t*)rwip_heap_non_ret, RWIP_CALC_HEAP_LEN_IN_BYTES(RWIP_HEAP_NON_RET_SIZE));
    ke_mem_init(KE_MEM_NON_RETENTION, (uint8_t*)g_rwip_heap_non_ret, g_rwip_heap_non_ret_size);

    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    #if (RW_DEBUG)
    // Initialize the debug process
    dbg_init(rwip_rst_state);
    #endif //(RW_DEBUG)
    #endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)

    //DONE TRANSPLANT  
    // Initialize RF 
    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    
    REG32(MODEM_BASE +0x59*4) |= 0xF0;
    rf_init(&rwip_rf);


	REG32(BASEBAND_REG_BASE +0x80) = 0x00500250; 
	llhwc_phy_prerx_flash();
    #endif //BT_EMB_PRESENT || BLE_EMB_PRESENT

    #if (DISPLAY_SUPPORT)
    // Initialize display module
    display_init();

    // Add some configuration information to display
    display_add_config();
    #endif //DISPLAY_SUPPORT

    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    // Initialize Diffie Hellman Elliptic Curve Algorithm
    #if (BLE_ECC)
    ecc_init(rwip_rst_state);
    #endif //(BLE_ECC)
    #endif // (BT_EMB_PRESENT || BLE_EMB_PRESENT)
   
    if(g_hci_transport_onff)
    {   
        // Initialize H4TL
        #if (H4TL_SUPPORT)
        #if (H4TL_NB_CHANNEL > 1)
        h4tl_init(1, rwip_eif_get(1));
        #endif // (H4TL_NB_CHANNEL > 1)
        h4tl_init(0, rwip_eif_get(0));
        #endif //(H4TL_SUPPORT)
    }
    #if (HCI_PRESENT)
    // Initialize the HCI
    hci_init(rwip_rst_state);
    #endif //HCI_PRESENT
    
    #if (AHI_TL_SUPPORT)
    // Initialize the Application Host Interface
    ahi_init();
    #endif //AHI_TL_SUPPORT
 
    #if (BLE_HOST_PRESENT)
    // Initialize BLE Host stack
    rwble_hl_init(rwip_rst_state);
    #endif //BLE_HOST_PRESENT

    #if (BT_EMB_PRESENT)
    // Initialize BT
    rwbt_init();
    #endif //BT_EMB_PRESENT

    #if (BLE_EMB_PRESENT)
    // Initialize BLE
    rwble_init(rwip_rst_state);
    #endif //BLE_EMB_PRESENT

    // Initialize IP core driver
    rwip_driver_init(rwip_rst_state);

    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    #if (RW_WLAN_COEX)
    rwip_wlcoex_set(1);
    #endif //(RW_WLAN_COEX)
    #endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)

    #if (BT_EMB_PRESENT || (BLE_EMB_PRESENT && !BLE_HOST_PRESENT))
    // If FW initializes due to FW reset, send the message to Host
    if(error != RESET_NO_ERROR)
    {
        if(error == RESET_TO_ROM || error == RESET_AND_LOAD_FW)
        {
            // Send platform reset command complete if requested by user
            dbg_platform_reset_complete(error);
        }
        else
        {
            // Allocate a message structure for hardware error event
            struct hci_hw_err_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_HW_ERR_EVT_CODE, hci_hw_err_evt);

            // Fill the HW error code
            switch(error)
            {
                case RESET_MEM_ALLOC_FAIL: evt->hw_code = CO_ERROR_HW_MEM_ALLOC_FAIL; break;
                default: ASSERT_INFO(0, error, 0); break;
            }

            // Send the message
            hci_send_2_host(evt);
        }
    }
    #endif //(BT_EMB_PRESENT || (BLE_EMB_PRESENT && !BLE_HOST_PRESENT))

    /*
     ************************************************************************************
     * Application initialization
     ************************************************************************************
     */
//    #if (BLE_APP_PRESENT)
//    // Initialize APP
//    if(g_app_init_branch)
//    {
//        app_init();
//    }   
//    #endif //BLE_APP_PRESENT

    // Move to IP first reset state
    rwip_rst_state = RWIP_1ST_RST;

    #if ((!BLE_HOST_PRESENT && BLE_EMB_PRESENT) || BT_EMB_PRESENT)
    // Make a full initialization in split-emb mode
    rwip_reset();
    #endif // ((!BLE_HOST_PRESENT && BLE_EMB_PRESENT) || BT_EMB_PRESENT)
}



void rwip_reset(void)
{
    // Disable interrupts until reset procedure is completed
    GLOBAL_INT_DISABLE();

    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    #if (RW_DEBUG)
    // Reset dbg
    dbg_init(rwip_rst_state);
    #endif //(RW_DEBUG)
    #endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)

    //Clear all message and timer pending
    ke_flush();

    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    // Reset Diffie Hellman Elliptic Curve Algorithm
    #if (BLE_ECC)
    ecc_init(rwip_rst_state);
    #endif //(BLE_ECC)
    #endif // (BT_EMB_PRESENT || BLE_EMB_PRESENT)

    #if (HCI_PRESENT)
    // Reset the HCI
    hci_init(rwip_rst_state);
    #endif //HCI_PRESENT

    #if (BLE_HOST_PRESENT)
    // Initialize BLE Host stack
    rwble_hl_init(rwip_rst_state);
    #endif //BLE_HOST_PRESENT

    #if (BT_EMB_PRESENT)
    if (rwip_rst_state == RWIP_RST)
    {
        // Reset BT
        rwbt_reset();
    }
    #endif //BT_EMB_PRESENT

    #if (BLE_EMB_PRESENT)
    // Reset BLE
    rwble_init(rwip_rst_state);
    #endif //BLE_EMB_PRESENT

    // Reset AES
    aes_init(rwip_rst_state);

    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    // Reset Scheduling blocks
    sch_arb_init(rwip_rst_state);
    sch_prog_init(rwip_rst_state);
    sch_plan_init(rwip_rst_state);
    sch_alarm_init(rwip_rst_state);
    sch_slice_init(rwip_rst_state);
    #endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)

    #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    // Initialize IP core driver
    rwip_driver_init(rwip_rst_state);
    #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    #if (RW_WLAN_COEX)
    rwip_wlcoex_set(1);
    #endif //(RW_WLAN_COEX)


    if (rwip_rst_state == RWIP_RST)
    {
        // Reset the RF
        rwip_rf.reset();
    }
    #endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)


    #if (DISPLAY_SUPPORT)
    // Restart display module
    display_resume();
    #endif //DISPLAY_SUPPORT

    // Move to normal IP reset state
    rwip_rst_state = RWIP_RST;
    
    // Restore interrupts once reset procedure is completed
    GLOBAL_INT_RESTORE();
}



