/**
 ****************************************************************************************
 *
 * @file wscc_task.h
 *
 * @brief Header file - Weight SCale ProfileCollector/Client Role Task.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 * $ Rev $
 *
 ****************************************************************************************
 */

#ifndef _WSCC_TASK_H_
#define _WSCC_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup WSCCTASK  Weight Scale Collector Task
 * @ingroup WSCC
 * @brief  Weight SCale ProfileProfile Collector Task
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "wsc_common.h"
#include "rwip_task.h" // Task definitions
#include "prf_types.h"

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Message IDs
/*@TRACE*/
enum wscc_msg_ids
{
    /// Enable the Profile Collector task - at connection
    WSCC_ENABLE_REQ = TASK_FIRST_MSG(TASK_ID_WSCC),
    /// Response to Enable the Profile Collector task - at connection
    WSCC_ENABLE_RSP,
   /// Read the Weight Scale Feature
    WSCC_RD_FEAT_CMD,
    /// Read the CCC of a measurement sensor characteristic
    WSCC_RD_MEAS_CCC_CMD,
    /// Write the CCC of a measurement sensor characteristic
    WSCC_WR_MEAS_CCC_CMD,
    /// Read the Weight Scale Feature
    WSCC_FEAT_IND,
    /// Read the CCC of a measurement sensor characteristic
    WSCC_MEAS_CCC_IND,
    /// Characteristic Measurement Indication from peer
    WSCC_MEAS_IND,
    /// Complete Event Information
    WSCC_CMP_EVT,
};


/// Weight Scale Service Characteristics
enum wscc_chars
{
    /// Features
    WSCC_CHAR_WSS_FEATURE,
    /// Measurement
    WSCC_CHAR_WSS_MEAS,

    WSCC_CHAR_WSS_MAX,
};

/// Weight Scale Service Characteristic Descriptors
enum wscc_descs
{
    /// Client config
    WSCC_DESC_WSS_MEAS_CCC,

    WSCC_DESC_WSS_MAX,
};

/*
 * API MESSAGE STRUCTURES
 ****************************************************************************************
 */

/**
 * Structure containing the characteristics handles, value handles and descriptors for
 * the Weight Scale Service
 */
struct wscc_wss_content
{
    /// service info
    struct prf_svc svc;

    /// Included service info - Body Composition Service..
    struct prf_incl_svc incl_svc;

    /// Characteristic info:
    ///  - Feature
    ///  - Measurement
    struct prf_char_inf chars[WSCC_CHAR_WSS_MAX];

    /// Descriptor handles:
    ///  - Client cfg
    struct prf_char_desc_inf descs[WSCC_DESC_WSS_MAX];
};

/// Parameters of the @ref WSCC_ENABLE_REQ message
struct wscc_enable_req
{
    /// Connection type
    uint8_t con_type;
    /// Existing handle values WSCC ES
    struct wscc_wss_content wss;
};

/// Parameters of the @ref WSCC_ENABLE_RSP message
struct wscc_enable_rsp
{
    /// status
    uint8_t status;
    /// Existing handle values WSCC ES
    struct wscc_wss_content wss;
};

/// Parameters of the @ref WSCC_RD_FEAT_CMD message
/// Read the Weight Scale Feature
struct wscc_rd_feat_cmd
{
    /// NULL
};

/// Parameters of the @ref WSCC_RD_MEAS_CCC_CMD message
/// Read the CCC of a measurement sensor characteristic
struct wscc_rd_meas_ccc_cmd
{
    /// NULL
};

/// Parameters of the @ref WSCC_WR_MEAS_CCC_CMD message
/// Write the CCC of a measurement sensor characteristic
struct wscc_wr_meas_ccc_cmd
{
    /// Client Characteristic Configuration read value
    uint16_t ccc;
};

/// Parameters of the @ref WSCC_FEAT_IND message
/// Read the Weight Scale Feature
struct wscc_feat_ind
{
    /// Weight Scale Feature read value
    uint32_t feature;
};

/// Parameters of the @ref WSCC_MEAS_CCC_IND message
/// Read the CCC of a measurement sensor characteristic
struct wscc_meas_ccc_ind
{
    /// Client Characteristic Configuration read value
    uint16_t ccc;
};

/// Parameters of the @ref WSCC_MEAS_IND message
/// Characteristic Measurement Indication from peer
struct wscc_meas_ind
{
    /// Presence of optional fields and measurement units
    uint8_t flags;
    /// Weight value
    uint16_t weight;
    /// Weight Resolution
    uint8_t wght_resol;
    /// Measurement Units
    uint8_t meas_u;
    /// Time stamp value
    struct prf_date_time time_stamp;
    /// User ID value
    uint8_t user_id;
    /// Body Mass Index
    uint16_t bmi;
    /// Height value
    uint16_t height;
    /// Height Resolution
    uint8_t hght_resol;
};

/// Parameters of the @ref WSCC_CMP_EVT message
struct wscc_cmp_evt
{
    /// Operation code
    uint8_t operation;
    /// Status
    uint8_t status;
};

/// @} WSCCTASK

#endif //(_WSCC_TASK_H_)
