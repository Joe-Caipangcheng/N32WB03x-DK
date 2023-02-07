#ifndef MESH_PROV_EVENTS_H__
#define MESH_PROV_EVENTS_H__

#include <stdint.h>
#include <stdbool.h>
#include "mesh_prov_types.h"
#include "mesh.h"
#include "mesh_prov_bearer.h"

typedef enum
{
    MESH_PROV_EVT_UNPROVISIONED_RECEIVED,
    MESH_PROV_EVT_LINK_ESTABLISHED,
    MESH_PROV_EVT_LINK_CLOSED,
    MESH_PROV_EVT_INVITE_RECEIVED,
    MESH_PROV_EVT_START_RECEIVED,
    MESH_PROV_EVT_OUTPUT_REQUEST,
    MESH_PROV_EVT_INPUT_REQUEST,
    MESH_PROV_EVT_STATIC_REQUEST,
    MESH_PROV_EVT_OOB_PUBKEY_REQUEST,
    MESH_PROV_EVT_CAPS_RECEIVED,
    MESH_PROV_EVT_COMPLETE,
    MESH_PROV_EVT_ECDH_REQUEST,
    MESH_PROV_EVT_FAILED
} mesh_prov_evt_type_t;

typedef struct
{
    uint8_t device_uuid[MESH_UUID_SIZE];
    bool gatt_supported;
    bool uri_hash_present;
    uint8_t uri_hash[MESH_BEACON_UNPROV_URI_HASH_SIZE];
    const mesh_rx_metadata_t * p_metadata;
} mesh_prov_evt_unprov_t;

typedef struct
{
    mesh_prov_ctx_t * p_context;
} mesh_prov_evt_link_established_t;

typedef struct
{
    mesh_prov_ctx_t * p_context;
    mesh_prov_link_close_reason_t close_reason;
} mesh_prov_evt_link_closed_t;

typedef struct
{
    mesh_prov_ctx_t * p_context;
    uint8_t attention_duration_s;
} mesh_prov_evt_invite_received_t;

typedef struct
{
    mesh_prov_ctx_t * p_context;
} mesh_prov_evt_start_received_t;

typedef struct
{
    mesh_prov_ctx_t * p_context;
    uint8_t action;
    uint8_t size;
} mesh_prov_evt_input_request_t;

typedef struct
{
    mesh_prov_ctx_t * p_context;
    uint8_t action;
    uint8_t size;
    const uint8_t * p_data;
} mesh_prov_evt_output_request_t;

typedef struct
{
    mesh_prov_ctx_t * p_context;
} mesh_prov_evt_static_request_t;

typedef struct
{
    mesh_prov_ctx_t * p_context;
    mesh_prov_oob_caps_t oob_caps;
} mesh_prov_evt_caps_received_t;

typedef struct
{
    mesh_prov_ctx_t * p_context;
    const uint8_t * p_devkey;
    const mesh_prov_provisioning_data_t * p_prov_data;
} mesh_prov_evt_complete_t;

typedef struct
{
    mesh_prov_ctx_t * p_context;
    mesh_prov_failure_code_t failure_code;
} mesh_prov_evt_failed_t;

typedef struct
{
    mesh_prov_ctx_t * p_context;
} mesh_prov_evt_oob_pubkey_request_t;

typedef struct
{
    mesh_prov_ctx_t * p_context;
    const uint8_t * p_peer_public;
    const uint8_t * p_node_private;
} mesh_prov_evt_ecdh_request_t;

typedef struct
{
    mesh_prov_evt_type_t type;
    union
    {
        mesh_prov_evt_unprov_t              unprov;
        mesh_prov_evt_link_established_t    link_established;
        mesh_prov_evt_link_closed_t         link_closed;
        mesh_prov_evt_invite_received_t     invite_received;
        mesh_prov_evt_start_received_t      start_received;
        mesh_prov_evt_input_request_t       input_request;
        mesh_prov_evt_output_request_t      output_request;
        mesh_prov_evt_static_request_t      static_request;
        mesh_prov_evt_oob_pubkey_request_t  oob_pubkey_request;
        mesh_prov_evt_caps_received_t       oob_caps_received;
        mesh_prov_evt_complete_t            complete;
        mesh_prov_evt_ecdh_request_t        ecdh_request;
        mesh_prov_evt_failed_t              failed;
    } params;
} mesh_prov_evt_t;


typedef void (*mesh_prov_evt_handler_cb_t)(const mesh_prov_evt_t * p_evt);




#endif /* MESH_PROV_EVENTS_H__ */
