#ifndef MESH_EVT_H__
#define MESH_EVT_H__

#include <stdbool.h>
#include "mesh.h"
#include "mesh_list.h"
#include "mesh_config_entry.h"
#include "mesh_config.h"

typedef enum
{
    MESH_EVT_MESSAGE_RECEIVED,
    MESH_EVT_TX_COMPLETE,
    MESH_EVT_IV_UPDATE_NOTIFICATION,
    MESH_EVT_KEY_REFRESH_NOTIFICATION,
    MESH_EVT_NET_BEACON_RECEIVED,
    MESH_EVT_HB_MESSAGE_RECEIVED,
    MESH_EVT_HB_SUBSCRIPTION_CHANGE,
    MESH_EVT_DFU_REQ_RELAY,
    MESH_EVT_DFU_REQ_SOURCE,
    MESH_EVT_DFU_START,
    MESH_EVT_DFU_END,
    MESH_EVT_DFU_BANK_AVAILABLE,
    MESH_EVT_DFU_FIRMWARE_OUTDATED,
    MESH_EVT_DFU_FIRMWARE_OUTDATED_NO_AUTH,
    MESH_EVT_FLASH_STABLE,
    MESH_EVT_RX_FAILED,
    MESH_EVT_SAR_FAILED,
    MESH_EVT_FLASH_FAILED,
    MESH_EVT_CONFIG_STABLE,
    MESH_EVT_CONFIG_STORAGE_FAILURE,
    MESH_EVT_CONFIG_LOAD_FAILURE,
    MESH_EVT_LPN_FRIEND_OFFER,
    MESH_EVT_LPN_FRIEND_UPDATE,
    MESH_EVT_LPN_FRIEND_REQUEST_TIMEOUT,
    MESH_EVT_LPN_FRIEND_POLL_COMPLETE,
    MESH_EVT_FRIENDSHIP_ESTABLISHED,
    MESH_EVT_FRIENDSHIP_TERMINATED,
    MESH_EVT_PROXY_STOPPED,
    MESH_EVT_DISABLED,
    MESH_EVT_FRIEND_REQUEST,
} mesh_evt_type_t;
typedef struct
{
    const uint8_t * p_buffer;
    uint16_t length;
    mesh_address_t src;
    mesh_address_t dst;
    mesh_secmat_t secmat;
    uint8_t ttl;
    const mesh_rx_metadata_t * p_metadata;
} mesh_evt_message_t;
typedef struct
{
    net_state_iv_update_t state;
    const uint8_t * p_network_id;
    uint32_t iv_index;
} mesh_evt_iv_update_notification_t;

typedef struct
{
    uint16_t subnet_index;
    const uint8_t * p_network_id;
    mesh_key_refresh_phase_t phase;
} mesh_evt_key_refresh_notification_t;
typedef struct
{
    mesh_tx_token_t token;
    timestamp_t timestamp;
} mesh_evt_tx_complete_t;
typedef enum
{
    MESH_RX_FAILED_REASON_REPLAY_CACHE_FULL
}mesh_rx_failed_reason_t;
typedef struct
{
    uint16_t src;
    uint8_t ivi : 1;
    mesh_rx_failed_reason_t reason;
}mesh_evt_rx_failed_t;
typedef enum
{
    MESH_SAR_CANCEL_REASON_TIMEOUT,
    MESH_SAR_CANCEL_REASON_RETRY_OVER,
    MESH_SAR_CANCEL_REASON_NO_MEM,
    MESH_SAR_CANCEL_BY_PEER,
    MESH_SAR_CANCEL_REASON_INVALID_FORMAT,
    MESH_SAR_CANCEL_PEER_STARTED_ANOTHER_SESSION,
    MESH_SAR_CANCEL_REASON_FRIENDSHIP_TERMINATED,
    MESH_SAR_CANCEL_REASON_FRIENDSHIP_ESTABLISHED,
    MESH_SAR_CANCEL_REASON_LPN_RX_NOT_COMPLETE
} mesh_sar_session_cancel_reason_t;
typedef struct
{
    mesh_tx_token_t token;
    mesh_sar_session_cancel_reason_t reason;
} mesh_evt_sar_failed_t;

typedef struct
{
    mesh_config_entry_id_t id; 
} mesh_evt_config_storage_failure_t;

typedef struct
{
    mesh_config_load_failure_t reason; 
    mesh_config_entry_id_t id;         
    const void * p_data;               
    uint32_t data_len;                 
} mesh_evt_config_load_failure_t;
typedef struct
{
    mesh_evt_type_t type;
    union {
        mesh_evt_message_t                  message;
        mesh_evt_tx_complete_t              tx_complete;
        mesh_evt_iv_update_notification_t   iv_update;
        mesh_evt_key_refresh_notification_t key_refresh;
        mesh_evt_config_storage_failure_t   config_storage_failure;
        mesh_evt_config_load_failure_t      config_load_failure;
    } params;
} mesh_evt_t;

typedef void (*mesh_evt_handler_cb_t)(const mesh_evt_t * p_evt);
typedef struct
{
    mesh_evt_handler_cb_t evt_cb;
    list_node_t node;
    bool is_removed;
} mesh_evt_handler_t;
void mesh_evt_handler_add(mesh_evt_handler_t * p_handler_params);
void mesh_evt_handler_remove(mesh_evt_handler_t * p_handler_params);

#endif /* MESH_EVT_H__ */

