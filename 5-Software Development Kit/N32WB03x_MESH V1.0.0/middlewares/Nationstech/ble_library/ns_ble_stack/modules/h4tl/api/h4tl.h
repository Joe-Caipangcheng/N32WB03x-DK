/**
 ****************************************************************************************
 *
 * @file h4tl.h
 *
 * @brief H4 UART Transport Layer header file.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef H4TL_H_
#define H4TL_H_

/**
 ****************************************************************************************
 * @addtogroup H4TL H4 UART Transport Layer
 * @ingroup H4TL
 * @brief H4 UART Transport Layer
 *
 * This module creates the abstraction between External UART driver and HCI generic functions
 * (designed for H4 UART transport layer).
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"  // stack configuration

#if (H4TL_SUPPORT)
#include "rwip.h"            // SW interface

#include <stdint.h>       // standard integer definition
#include <stdbool.h>      // standard boolean definition
#include "ahi.h"


/// Size of the RX Buffer that can both receive the header and valid packet to exit from out of sync mode
#if (AHI_TL_SUPPORT)
#define RX_TMP_BUFF_SIZE     AHI_RESET_MSG_LEN
#elif (HCI_TL_SUPPORT)
#define RX_TMP_BUFF_SIZE     HCI_RESET_MSG_LEN
#endif // (AHI_TL_SUPPORT) or (HCI_TL_SUPPORT)


///H4TL RX states
enum H4TL_STATE_RX
{
    ///H4TL RX Start State - receive message type
    H4TL_STATE_RX_START,
    ///H4TL RX Header State - receive message header
    H4TL_STATE_RX_HDR,
    ///H4TL RX Header State - receive (rest of) message payload
    H4TL_STATE_RX_PAYL,
    ///H4TL RX Out Of Sync state - receive message type
    H4TL_STATE_RX_OUT_OF_SYNC
};



///H4TL queue type
enum H4TL_QUEUE
{
    /// HCI/AHI Tx Queue
    H4TL_TX_QUEUE_MSG,

#if (BLE_ISOOHCI)
    /// ISO Tx Queue
    H4TL_TX_QUEUE_ISO,
#endif // (BLE_ISOOHCI)

#if (TRACER_PRESENT)
    /// Tracer Message Queue
    H4TL_TX_QUEUE_TRACER,
#endif // (TRACER_PRESENT)

    /// Maximum number of TX queue
    H4TL_TX_QUEUE_MAX,

    /// Queue in Idle mode
    H4TL_TX_QUEUE_IDLE  = 0xFF,
};
/*
 * STRUCTURES DEFINITION
 ****************************************************************************************
 */

/// Information about message under transmission
struct h4tl_tx_info
{
    /// message buffer
    uint8_t* buf;
    /// Tx callback
    void (*callback)(void);
    /// buffer length
    uint16_t buf_len;
    /// Priority
    uint8_t prio;
};


/// H4TL Environment context structure
struct h4tl_env_tag
{
    /// pointer to External interface api

    const struct rwip_eif_api* ext_if;
    ///Pointer to space reserved for received payload.
    uint8_t* curr_payl_buff;
    /// Ensure that array is 32bits aligned
    /// Latest received message header, or used to receive a message allowing to exit from out of sync
    uint8_t rx_buf[RX_TMP_BUFF_SIZE];

    ///Rx state - can be receiving message type, header, payload or error
    uint8_t rx_state;
    ///Latest received message type: CMD/EVT/ACL.
    uint8_t rx_type;
    /// Transport layer interface
    uint8_t tl_itf;

    /// Maximum Tx queue
    struct h4tl_tx_info tx_queue[H4TL_TX_QUEUE_MAX];

    /// Information about on-going transmission
    uint8_t tx_active_queue;
};
/*
 * DEFINES
 ****************************************************************************************
 */

/// Size of the logical channel identifier for H4 messages
#define H4TL_LOGICAL_CHANNEL_LEN     (1)

/**
 * Number of H4TL interfaces
 *
 *  * NB=2: AHI and HCI: for Host-only stack with external app
 *       - HCI has index 0
 *       - AHI has index 1
 *  * NB=1: AHI or HCI: for all other partitions
 *
 * Note: it is not possible to have no channel (H4TL must not be included in build in this case)
 */
#if (!BLE_EMB_PRESENT && HCI_TL_SUPPORT && AHI_TL_SUPPORT)
#define H4TL_NB_CHANNEL     2
#else // (!BLE_EMB_PRESENT && HCI_TL_SUPPORT && AHI_TL_SUPPORT)
#define H4TL_NB_CHANNEL     1
#endif // (!BLE_EMB_PRESENT && HCI_TL_SUPPORT && AHI_TL_SUPPORT)


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
 * @brief H4TL transport initialization.
 *
 * Puts the External Interface driver in reception, waiting for simple 1 byte message type. Space for
 * reception is allocated with ke_msg_alloc and the pointer is handed to env.rx. RX
 * interrupt is enabled.
 *
 * @param[in] tl_type  Transport Layer Interface (@see enum h4tl_itf)
 * @param[in] eif      External interface API
 *
 *****************************************************************************************
 */
void h4tl_init(uint8_t tl_type, const struct rwip_eif_api* eif);

static void h4tl_read_start(struct h4tl_env_tag* env);
/**
 ****************************************************************************************
 * @brief H4TL write function.
 *
 * @param[in] type  Type of the buffer to be transmitted. It can take one of the following
 *                  values:
 *      - @ref HCI_EVT_MSG_TYPE for event message
 *      - @ref HCI_ACL_MSG_TYPE for ACL data
 *      - @ref HCI_SYNC_MSG_TYPE for synchronous data
 *
 * @param[in] buf   Pointer to the buffer to be transmitted. @note The buffer passed as
 *  parameter must have one free byte before the first payload byte, so that the H4TL
 *  module can put the type byte as first transmitted data.
 *
 * @param[in] len   Length of the buffer to be transmitted.
 * @param[in] tx_callback   Callback for indicating the end of transfer
 *****************************************************************************************
 */
void h4tl_write(uint8_t type, uint8_t *buf, uint16_t len, void (*tx_callback)(void));

/**
 ****************************************************************************************
 * @brief Start External Interface input flow
 *
 *****************************************************************************************
 */
void h4tl_start(void);

/**
 ****************************************************************************************
 * @brief Stop External Interface input flow if possible
 *
 * @return true if External Interface flow was stopped, false otherwise
 *****************************************************************************************
 */
//bool h4tl_stop(void);

void h4tl_read_start_next(void);

#endif //H4TL_SUPPORT

/// @} H4TL

#endif // H4TL_H_
