/**
 ****************************************************************************************
 *
 * @file ahi_task.h
 *
 * @brief This file contains definitions related to the Application Host Interface
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef AHI_TASK_H_
#define AHI_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup AHI Application Host Interface
 *@{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (AHI_TL_SUPPORT)

#include "rwip_task.h"       // Task definition
#include "rwble_hl.h"        // BLE HL default include

/*
 * INSTANCES
 ****************************************************************************************
 */
/// Maximum number of instances of the AHI task
#define AHI_IDX_MAX 1




#define LEN_PATCH_VERSION       6



/*
 * STATES
 ****************************************************************************************
 */
/// Possible states of the AHI task
enum AHI_STATE
{
    /// TX IDLE state
    AHI_TX_IDLE,
    /// TX ONGOING state
    AHI_TX_ONGOING,
    /// Number of states.
    AHI_STATE_MAX
};

/*
 * MESSAGES
 ****************************************************************************************
 */
/// Message API of the AHI task
/*@TRACE*/
enum ahi_msg_id
{
    // Module basic command
    AHI_CMD_RESET                   = TASK_FIRST_MSG(TASK_ID_AHI), 
    AHI_CMD_SLEEP                   = TASK_FIRST_MSG(TASK_ID_AHI) + 0x01,
    AHI_CMD_SET_UART_BAUD           = TASK_FIRST_MSG(TASK_ID_AHI) + 0x02,
    AHI_CMD_GET_VERSION             = TASK_FIRST_MSG(TASK_ID_AHI) + 0x03,
    AHI_CMD_GET_NAME                = TASK_FIRST_MSG(TASK_ID_AHI) + 0x04,
    AHI_CMD_IRQ_RISE_TIME           = TASK_FIRST_MSG(TASK_ID_AHI) + 0x05,
    AHI_CMD_BYPASS                  = TASK_FIRST_MSG(TASK_ID_AHI) + 0x06,
    
    
    // General
    AHI_CMD_MAC_ADDR                = TASK_FIRST_MSG(TASK_ID_AHI) + 0x20,
    AHI_CMD_TX_POWER                = TASK_FIRST_MSG(TASK_ID_AHI) + 0x21,
    AHI_CMD_GET_STATE               = TASK_FIRST_MSG(TASK_ID_AHI) + 0x22,
    AHI_CMD_MTU                     = TASK_FIRST_MSG(TASK_ID_AHI) + 0x23,
    AHI_CMD_CONN_PARAM              = TASK_FIRST_MSG(TASK_ID_AHI) + 0x24,
    AHI_CMD_DISCONNECT              = TASK_FIRST_MSG(TASK_ID_AHI) + 0x25,
    AHI_CMD_RSSI                    = TASK_FIRST_MSG(TASK_ID_AHI) + 0x26,
    AHI_CMD_DLE                     = TASK_FIRST_MSG(TASK_ID_AHI) + 0x27,
    AHI_CMD_SEC_PARAM               = TASK_FIRST_MSG(TASK_ID_AHI) + 0x28,
    AHI_CMD_BOND_INFO               = TASK_FIRST_MSG(TASK_ID_AHI) + 0x29,
    AHI_CMD_FREQ_TEST               = TASK_FIRST_MSG(TASK_ID_AHI) + 0x2A,
    
    // BLE Peripheral
    AHI_CMD_ADV_DATA                = TASK_FIRST_MSG(TASK_ID_AHI) + 0x40,
    AHI_CMD_SCAN_RSP_DATA           = TASK_FIRST_MSG(TASK_ID_AHI) + 0x41,
    AHI_CMD_ADV_PARAM               = TASK_FIRST_MSG(TASK_ID_AHI) + 0x42,
    AHI_CMD_ADV_START_CONTROL       = TASK_FIRST_MSG(TASK_ID_AHI) + 0x43,
    AHI_CMD_SEC_REQ                 = TASK_FIRST_MSG(TASK_ID_AHI) + 0x44,
    AHI_CMD_CFG_SVC                 = TASK_FIRST_MSG(TASK_ID_AHI) + 0x45,
    AHI_CMD_SET_READ_REQ_DATA       = TASK_FIRST_MSG(TASK_ID_AHI) + 0x46,
    AHI_CMD_SET_IND_NTF_DATA        = TASK_FIRST_MSG(TASK_ID_AHI) + 0x47,
    AHI_CMD_SET_WRITE_REQ_CFM       = TASK_FIRST_MSG(TASK_ID_AHI) + 0x48,
    
    // BLE Central
    AHI_CMD_SCAN_PARAM              = TASK_FIRST_MSG(TASK_ID_AHI) + 0x60,
    AHI_CMD_SCAN_START_CONTROL      = TASK_FIRST_MSG(TASK_ID_AHI) + 0x61,            
    AHI_CMD_CONNECT                 = TASK_FIRST_MSG(TASK_ID_AHI) + 0x62,
    AHI_CMD_START_PAIR              = TASK_FIRST_MSG(TASK_ID_AHI) + 0x63,
    AHI_CMD_GET_SVC                 = TASK_FIRST_MSG(TASK_ID_AHI) + 0x64,
    AHI_CMD_READ                    = TASK_FIRST_MSG(TASK_ID_AHI) + 0x65,   
    AHI_CMD_WRITE                   = TASK_FIRST_MSG(TASK_ID_AHI) + 0x66,  
    
    // CMD store and trigger
    
    // MCU Peripheral
    
    // Closed to user
    AHI_CMD_DEBUG_READ_REG          = TASK_FIRST_MSG(TASK_ID_AHI) + 0xF0,
    AHI_CMD_DEBUG_WRITE_REG         = TASK_FIRST_MSG(TASK_ID_AHI) + 0xF1,
    AHI_RESERVED                    = TASK_FIRST_MSG(TASK_ID_AHI) + 0xF2,
    
    AHI_MSG_ID_LAST
};




/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */

#endif //AHI_TL_SUPPORT

/// @} AHI

#endif // AHI_TASK_H_

