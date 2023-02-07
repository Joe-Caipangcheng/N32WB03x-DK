#ifndef CONFIG_SERVER_EVENTS_H__
#define CONFIG_SERVER_EVENTS_H__

#include <stdint.h>
#include "device_state_manager.h"
#include "access.h"
#include "heartbeat.h"

typedef enum
{
    CONFIG_SERVER_EVT_APPKEY_ADD,
    CONFIG_SERVER_EVT_APPKEY_UPDATE,
    CONFIG_SERVER_EVT_MODEL_PUBLICATION_SET,
    CONFIG_SERVER_EVT_APPKEY_DELETE,
    CONFIG_SERVER_EVT_BEACON_SET,
    CONFIG_SERVER_EVT_DEFAULT_TTL_SET,
    CONFIG_SERVER_EVT_FRIEND_SET,
    CONFIG_SERVER_EVT_GATT_PROXY_SET,
    CONFIG_SERVER_EVT_KEY_REFRESH_PHASE_SET,
    CONFIG_SERVER_EVT_MODEL_PUBLICATION_VIRTUAL_ADDRESS_SET,
    CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_ADD,
    CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_DELETE,
    CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_DELETE_ALL,
    CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_OVERWRITE,
    CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_ADD,
    CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_DELETE,
    CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_OVERWRITE,
    CONFIG_SERVER_EVT_NETWORK_TRANSMIT_SET,
    CONFIG_SERVER_EVT_RELAY_SET,
    CONFIG_SERVER_EVT_LOW_POWER_NODE_POLLTIMEOUT_SET,
    CONFIG_SERVER_EVT_HEARTBEAT_PUBLICATION_SET,
    CONFIG_SERVER_EVT_HEARTBEAT_SUBSCRIPTION_SET,
    CONFIG_SERVER_EVT_MODEL_APP_BIND,
    CONFIG_SERVER_EVT_MODEL_APP_UNBIND,
    CONFIG_SERVER_EVT_NETKEY_ADD,
    CONFIG_SERVER_EVT_NETKEY_DELETE,
    CONFIG_SERVER_EVT_NETKEY_UPDATE,
    CONFIG_SERVER_EVT_NODE_IDENTITY_SET,
    CONFIG_SERVER_EVT_NODE_RESET
} config_server_evt_type_t;



typedef struct
{
    dsm_handle_t appkey_handle;
} config_server_evt_appkey_add_t;

typedef struct
{
    uint8_t retransmit_count;
    uint8_t interval_steps;
} config_server_evt_network_transmit_set_t;



typedef struct
{
    access_model_handle_t model_handle;
    dsm_handle_t appkey_handle;
} config_server_evt_model_app_bind_t;
typedef struct
{
    access_model_handle_t model_handle;
    dsm_handle_t appkey_handle;
} config_server_evt_model_app_unbind_t;
typedef struct
{
    access_model_handle_t model_handle;
} config_server_evt_model_publication_set_t;

typedef struct
{
    access_model_handle_t model_handle;
    dsm_handle_t address_handle;
} config_server_evt_model_subscription_add_t;
typedef struct
{
    uint8_t default_ttl;
} config_server_evt_default_ttl_set_t;
typedef struct
{
    const heartbeat_subscription_state_t * p_subscription_state;
} config_server_evt_heartbeat_subscription_set_t;
typedef struct
{
    const heartbeat_publication_state_t * p_publication_state;
} config_server_evt_heartbeat_publication_set_t;
typedef struct
{
    bool enabled;
    uint8_t retransmit_count;
    uint8_t interval_steps;
} config_server_evt_relay_set_t;
typedef struct
{
    access_model_handle_t model_handle;
    dsm_handle_t address_handle;
} config_server_evt_model_subscription_delete_t;
typedef struct
{
    dsm_handle_t appkey_handle;
} config_server_evt_appkey_delete_t;


typedef struct
{
    config_server_evt_type_t type;
    union
    {
        config_server_evt_appkey_add_t appkey_add;
        config_server_evt_network_transmit_set_t network_transmit_set;
        config_server_evt_model_app_bind_t model_app_bind;
        config_server_evt_model_app_unbind_t model_app_unbind; 
        config_server_evt_model_publication_set_t model_publication_set;
        config_server_evt_model_subscription_add_t model_subscription_add;
        config_server_evt_default_ttl_set_t default_ttl_set;
        config_server_evt_heartbeat_publication_set_t heartbeat_publication_set;
        config_server_evt_heartbeat_subscription_set_t heartbeat_subscription_set;
        config_server_evt_relay_set_t relay_set;
        config_server_evt_model_subscription_delete_t model_subscription_delete;
        config_server_evt_appkey_delete_t appkey_delete;
        
    } params;
} config_server_evt_t;



typedef void (*config_server_evt_cb_t)(const config_server_evt_t * p_evt);


#endif /* CONFIG_SERVER_EVENTS_H__ */
