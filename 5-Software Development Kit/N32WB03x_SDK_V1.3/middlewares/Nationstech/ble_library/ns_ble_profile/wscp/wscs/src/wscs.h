/**
 ****************************************************************************************
 *
 * @file wscs.h
 *
 * @brief Header file - Weight SCale Profile Sensor.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 * $ Rev $
 *
 ****************************************************************************************
 */

#ifndef _WSCS_H_
#define _WSCS_H_

/**
 ****************************************************************************************
 * @addtogroup WSCS Weight SCale Profile Sensor
 * @ingroup CSCP
 * @brief Weight SCale Profile Sensor
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_WSC_SERVER)
#include "wsc_common.h"
#include "wscs_task.h"

#include "prf_types.h"
#include "prf.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximum number of Weight SCale Profile Sensor role task instances
#define WSCS_IDX_MAX                (BLE_CONNECTION_MAX)
/// Feature value size 32 bits
#define WSCS_FEAT_VAL_SIZE          (4)
/// Measurement Indication size (plus 3 bytes for meas_u, mass_resol and hght_resol)
#define WSCS_MEAS_IND_SIZE          (sizeof(struct wscs_meas_indicate_cmd) + 3)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Possible states of the WSCS task
enum wscs_states
{
    /// not connected state
    WSCS_FREE,
    /// idle state
    WSCS_IDLE,

    /// indicate 
    WSCS_OP_INDICATE,

    /// Number of defined states.
    ENVS_STATE_MAX
};

/// Weight SCale Service - Attribute List
enum wscs_att_list
{
    /// Weight SCale Service
    WSCS_IDX_SVC,
    /// Included Body Composition Service
    WSCS_IDX_INC_SVC,
    /// Weight SCale Feature Characteristic
    WSCS_IDX_FEAT_CHAR,
    WSCS_IDX_FEAT_VAL,
    /// Weight SCale Measurement Characteristic
    WSCS_IDX_MEAS_CHAR,
    WSCS_IDX_MEAS_IND,
    /// CCC Descriptor
    WSCS_IDX_MEAS_CCC,

    /// Number of attributes
    WSCS_IDX_NB,
};


/*
 * STRUCTURES
 ****************************************************************************************
 */

/// Weight SCale Profile Sensor environment variable
struct wscs_env_tag
{
    /// profile environment
    prf_env_t prf_env;

    /// Weight SCale Service Start Handle
    uint16_t shdl;
    /// Feature configuration
    uint32_t feature;
    /// BCS Service Included
    uint8_t bcs_included;
    /// CCC for each connections
    uint8_t prfl_ind_cfg[BLE_CONNECTION_MAX];

    /// State of different task instances
    ke_state_t state[WSCS_IDX_MAX];

};

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
 
/**
 ****************************************************************************************
 * @brief Inform the APP that a procedure has been completed
 *
 * @param[in] p_wscs_env    Weight SCale Sensor environment variable
 * @param[in] conidx        Connection index
 * @param[in] operation     Operation code
 * @param[in] status        Operation status
 ****************************************************************************************
 */
void wscs_send_cmp_evt(struct wscs_env_tag *p_wscs_env, uint8_t conidx, uint8_t operation, uint8_t status);

/**
 ****************************************************************************************
 * @brief Retrieve WSCP service profile interface
 *
 * @return Weight Scale service profile interface
 ****************************************************************************************
 */
const struct prf_task_cbs* wscs_prf_itf_get(void);

/**
 ****************************************************************************************
 * @brief Get Weight Resolution from feature value
 *
 * @param[in] feature Weight Scale Feature value
 *
 * @return Mass Resolution
 ****************************************************************************************
 */
uint8_t wscs_get_wght_resol(uint32_t feature);

/**
 ****************************************************************************************
 * @brief Get Height Resolution from feature value
 *
 * @param[in] feature Weight Scale Feature value
 *
 * @return Height Resolution
 ****************************************************************************************
 */
uint8_t wscs_get_hght_resol(uint32_t feature);

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
void wscs_task_init(struct ke_task_desc *p_task_desc);

#endif //(BLE_WSC_SERVER)

/// @} WSCS

#endif //(_WSCS_H_)
