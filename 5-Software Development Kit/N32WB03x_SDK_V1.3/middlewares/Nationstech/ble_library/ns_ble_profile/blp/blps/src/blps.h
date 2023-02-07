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
 * @file blps.h
 * @author Nations Firmware Team
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */


#ifndef _BLPS_H_
#define _BLPS_H_

/**
 ****************************************************************************************
 * @addtogroup BLPS Blood Pressure Profile Sensor
 * @ingroup BLP
 * @brief Blood Pressure Profile Sensor
 *
 * Blood pressure sensor (BPS) profile provides functionalities to upper layer module
 * application. The device using this profile takes the role of Blood pressure sensor.
 *
 * The interface of this role to the Application is:
 *  - Enable the profile role (from APP)
 *  - Disable the profile role (from APP)
 *  - Notify peer device during Blood Pressure measurement (from APP)
 *  - Indicate measurements performed to peer device (from APP)
 *
 * Profile didn't manages multiple users configuration and storage of offline measurements.
 * This must be handled by application.
 *
 * Blood Pressure Profile Sensor. (BLPS): A BLPS (e.g. PC, phone, etc)
 * is the term used by this profile to describe a device that can perform blood pressure
 * measurement and notify about on-going measurement and indicate final result to a peer
 * BLE device.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "blp_common.h"

#if (BLE_BP_SENSOR)

#include "prf_types.h"
#include "prf.h"
#include "blps_task.h"
#include "attm.h"

/*
 * DEFINES
 ****************************************************************************************
 */
///Maximum number of Blood Pressure task instances
#define BLPS_IDX_MAX     0x01

/// Maximum notification length
#define BLPS_BP_MEAS_MAX_LEN            (19)
#define BLPS_RACP_MAX_LEN               (20)

///BPS Configuration Flag Masks
#define BLPS_MANDATORY_MASK              (0x003F)
#define BLPS_INTM_CUFF_PRESS_MASK        (0x01C0)
#define BLPS_RACP_MASK                   (0X0E00)

/// indication/notification config mask
#define BLPS_NTFIND_MASK                 (0x0E)

/*
 * MACROS
 ****************************************************************************************
 */

/// Possible states of the BLPS task
enum
{
    /// Idle state
    BLPS_IDLE,
    /// Busy state
    BLPS_BUSY,

    /// Number of defined states.
    BLPS_STATE_MAX
};

///Attributes State Machine
enum
{
    BPS_IDX_SVC,

    BPS_IDX_BP_MEAS_CHAR,
    BPS_IDX_BP_MEAS_VAL,
    BPS_IDX_BP_MEAS_IND_CFG,

    BPS_IDX_BP_FEATURE_CHAR,
    BPS_IDX_BP_FEATURE_VAL,

    BPS_IDX_INTM_CUFF_PRESS_CHAR,
    BPS_IDX_INTM_CUFF_PRESS_VAL,
    BPS_IDX_INTM_CUFF_PRESS_NTF_CFG,
    
    BPS_IDX_RACP_CHAR,
    BPS_IDX_RACP_VAL,
    BPS_IDX_RACP_IND_CFG,

    BPS_IDX_NB,
};

///Characteristic Codes
enum
{
    BPS_BP_MEAS_CHAR,
    BPS_INTM_CUFF_MEAS_CHAR,
    BPS_BP_FEATURE_CHAR,
    BPS_RACP_CHAR,
};

/// Database Configuration Bit Field Flags
enum blps_db_config_bf
{
    /// support of Intermediate Cuff Pressure
    BLPS_INTM_CUFF_PRESS_SUP_POS = 0,
    BLPS_INTM_CUFF_PRESS_SUP_BIT = CO_BIT(BLPS_INTM_CUFF_PRESS_SUP_POS),
    
    /// support of Record Access Control Point
    BLPS_RACP_SUP_POS = 1,
    BLPS_RACP_SUP_BIT = CO_BIT(BLPS_RACP_SUP_POS),
};

/// Indication/notification configuration (put in feature flag to optimize memory usage)
enum blps_indntf_config_bf
{
    /// Bit used to know if blood pressure measurement indication is enabled
    BLPS_BP_MEAS_IND_CFG_POS = 1,
    BLPS_BP_MEAS_IND_CFG_BIT = CO_BIT(BLPS_BP_MEAS_IND_CFG_POS),

    /// Bit used to know if cuff pressure measurement notification is enabled
    BLPS_INTM_CUFF_PRESS_NTF_CFG_POS = 2,
    BLPS_INTM_CUFF_PRESS_NTF_CFG_BIT = CO_BIT(BLPS_INTM_CUFF_PRESS_NTF_CFG_POS),
    
    /// Bit used to know if record access cotrol point notification is enabled
    BLPS_RACP_IND_CFG_POS = 3,
    BLPS_RACP_IND_CFG_BIT = CO_BIT(BLPS_RACP_IND_CFG_POS),
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Blood Pressure Profile Sensor environment variable per connection
struct blps_cnx_env
{
    /// Profile Notify/Indication Flags
    uint8_t prfl_ntf_ind_cfg;
};

/// Blood Pressure Profile Sensor environment variable
struct blps_env_tag
{
    /// profile environment
    prf_env_t prf_env;
    /// Blood Pressure Service Start Handle
    uint16_t shdl;
    /// Feature Configuration Flags
    uint16_t features;
    /// Profile configuration flags
    uint8_t prfl_cfg;
    /// Event (notification/indication) config
    uint8_t evt_cfg;
    /// Environment variable pointer for each connections
    uint8_t prfl_ntf_ind_cfg[BLE_CONNECTION_MAX];
    /// State of different task instances
    ke_state_t state[BLPS_IDX_MAX];
};

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Retrieve BLP service profile interface
 *
 * @return BLP service profile interface
 ****************************************************************************************
 */
const struct prf_task_cbs* blps_prf_itf_get(void);

/**
 ****************************************************************************************
 * @brief Pack Blood Pressure measurement value
 *
 * @param[out] p_packed_bp Pointer of the packed data of Blood Pressure Measurement information
 * @param[in] p_meas_val Blood Pressure measurement value
 * @return size of packed value
 ****************************************************************************************
 */
uint8_t blps_pack_meas_value(uint8_t *p_packed_bp, const struct bps_bp_meas* p_meas_val);

/**
 ****************************************************************************************
 * @brief Send an BLPS_CMP_EVT message to a requester.
 *
 * @param[in] p_blps_env        BLP Sensor environment variable
 * @param[in] conidx            Connection index
 * @param[in] operation         Code of the completed operation
 * @param[in] operation_code    Code of the operation
 * @param[in] status            Status of the request
 ****************************************************************************************
 */
void blps_send_cmp_evt(struct blps_env_tag *p_blps_env, uint8_t conidx, uint8_t operation, uint8_t operation_code, uint8_t status);

/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * Initialize task handler
 *
 * @param p_task_desc Task descriptor to fill
 ****************************************************************************************
 */
void blps_task_init(struct ke_task_desc *p_task_desc);


#endif /* #if (BLE_BP_SENSOR) */

/// @} BLPS

#endif /* _BLPS_H_ */
