#ifndef CONFIG_MESSAGES_H__
#define CONFIG_MESSAGES_H__

#include <stddef.h>
#include <stdint.h>
#include "access.h"


#define CONFIG_RETRANSMIT_COUNT_MAX          ((1 << 3) - 1)
#define CONFIG_RETRANSMIT_INTERVAL_STEPS_MAX ((1 << 5) - 1)
#define CONFIG_RETRANSMIT_INTERVAL_STEP_TO_MS(STEP) ((STEP) * 10)
#define CONFIG_RETRANSMIT_INTERVAL_MS_TO_STEP(MS)   ((MS) / 10)


typedef struct __attribute((packed))
{
    uint8_t key_id_1_lsb;     
    uint8_t key_id_1_msb : 4; 
    uint8_t key_id_2_lsb : 4; 
    uint8_t key_id_2_msb;     
} config_msg_key_index_24_t;


static inline void config_msg_key_index_24_set(config_msg_key_index_24_t * p_idx24, uint16_t key_id_1, uint16_t key_id_2)
{
    p_idx24->key_id_1_lsb = key_id_1 & 0xff;
    p_idx24->key_id_1_msb = (key_id_1 >> 8) & 0xf;

    p_idx24->key_id_2_lsb = key_id_2 & 0xf;
    p_idx24->key_id_2_msb = (key_id_2 >> 4) & 0xff;
}

static inline void config_msg_key_index_24_get(const config_msg_key_index_24_t * p_idx24, uint16_t * p_key_id_1, uint16_t * p_key_id_2)
{
    if (p_key_id_1 != NULL)
    {
        *p_key_id_1 = p_idx24->key_id_1_lsb | (p_idx24->key_id_1_msb << 8);
    }

    if (p_key_id_2 != NULL)
    {
        *p_key_id_2 = p_idx24->key_id_2_lsb | (p_idx24->key_id_2_msb << 4);
    }
}

#define CONFIG_MSG_KEY_INDEX_12_MASK    0x0fff

typedef uint16_t config_msg_key_index_12_t;
typedef union __attribute((packed))
{
    struct __attribute((packed))
    {
        uint16_t model_id;     
    } sig;
    struct __attribute((packed))
    {
        uint16_t company_id;   
        uint16_t model_id;     
    } vendor;
} config_model_id_t;
static inline void config_msg_model_id_set(config_model_id_t * p_dst, const access_model_id_t * p_src, bool is_sig)
{
    if (is_sig)
    {
        p_dst->sig.model_id = p_src->model_id;
    }
    else
    {
        p_dst->vendor.model_id = p_src->model_id;
        p_dst->vendor.company_id = p_src->company_id;
    }
}
typedef struct __attribute((packed))
{
    config_msg_key_index_24_t key_indexes; 
    uint8_t appkey[MESH_KEY_SIZE];     
} config_msg_appkey_add_t;
typedef struct __attribute((packed))
{
    config_msg_key_index_24_t key_indexes; 
    uint8_t appkey[MESH_KEY_SIZE];     
} config_msg_appkey_update_t;
typedef struct __attribute((packed))
{
    config_msg_key_index_24_t key_indexes; 
} config_msg_appkey_delete_t;
typedef struct __attribute((packed))
{
    uint8_t status;                        
    config_msg_key_index_24_t key_indexes; 
} config_msg_appkey_status_t;
typedef struct __attribute((packed))
{
    config_msg_key_index_12_t netkey_index;
} config_msg_appkey_get_t;
typedef struct __attribute((packed))
{
    uint8_t status;                         
    config_msg_key_index_12_t netkey_index; 
    uint8_t packed_appkey_indexes[];        
} config_msg_appkey_list_t;
typedef struct __attribute((packed))
{
    uint8_t ttl;            
} config_msg_default_ttl_set_t;
typedef struct __attribute((packed))
{
    uint8_t ttl;           
} config_msg_default_ttl_status_t;
typedef enum
{
    CONFIG_NET_BEACON_STATE_DISABLED = 0,   
    CONFIG_NET_BEACON_STATE_ENABLED = 1     
} config_net_beacon_state_t;
typedef struct
{
    uint8_t beacon_state;      /**< Beacon state. */
} config_msg_net_beacon_set_t;
typedef struct __attribute((packed))
{
    uint8_t beacon_state;      /**< Beacon state. */
} config_msg_net_beacon_status_t;
typedef struct __attribute((packed))
{
    uint16_t appkey_index : 12;                   
    uint16_t credential_flag : 1;                 
    uint16_t rfu : 3;                             
    uint8_t  publish_ttl;                         
    uint8_t  publish_period;                      
    uint8_t  retransmit_count : 3;                
    uint8_t  retransmit_interval : 5;             
    config_model_id_t  model_id;                  
} config_publication_params_t;
typedef struct __attribute((packed))
{
    uint16_t element_address;   
    config_model_id_t model_id; 
} config_msg_publication_get_t;
typedef struct __attribute((packed))
{
    uint16_t element_address;          
    uint16_t publish_address;          
    config_publication_params_t state; 
} config_msg_publication_set_t;
typedef struct __attribute((packed))
{
    uint16_t element_address;                  
    uint8_t  publish_uuid[MESH_UUID_SIZE]; 
    config_publication_params_t state;         
} config_msg_publication_virtual_set_t;
typedef struct __attribute((packed))
{
    uint8_t status;                    
    uint16_t element_address;          
    uint16_t publish_address;          
    config_publication_params_t state; 
} config_msg_publication_status_t;
typedef struct __attribute((packed))
{
    uint16_t          element_address; /**< Address of the element. */
    uint16_t          address;         /**< Address to subscribe to. */
    config_model_id_t model_id;        /**< ID of the model. */
} config_msg_subscription_add_del_owr_t;
typedef struct __attribute((packed))
{
    uint16_t          element_address;                  /**< Address of the element. */
    uint8_t           virtual_uuid[MESH_UUID_SIZE]; /**< Label UUID for the virtual address to subscribe to. */
    config_model_id_t model_id;                         /**< ID of the model. */
} config_msg_subscription_virtual_add_del_owr_t;
typedef struct __attribute((packed))
{
    uint16_t          element_address; /**< Address of the element. */
    config_model_id_t model_id;        /**< ID of the model. */
} config_msg_subscription_delete_all_t;
typedef struct __attribute((packed))
{
    uint8_t           status;          /**< Status code. */
    uint16_t          element_address; /**< Address of the element. */
    uint16_t          address;         /**< Address that the model was subscribed to. */
    config_model_id_t model_id;        /**< ID of the model. */
} config_msg_subscription_status_t;
typedef struct __attribute((packed))
{
    config_msg_key_index_12_t netkey_index;              /**< Network key index. */
    uint8_t                   netkey[MESH_KEY_SIZE]; /**< Network key contents. */
} config_msg_netkey_add_update_t;
typedef struct __attribute((packed))
{
    config_msg_key_index_12_t netkey_index; /**< Network key index. */
} config_msg_netkey_delete_t;
typedef struct __attribute((packed))
{
    uint8_t status;                         /**< Status code. */
    config_msg_key_index_12_t netkey_index; /**< Network key index.*/
} config_msg_netkey_status_t;
typedef enum
{
    CONFIG_GATT_PROXY_STATE_RUNNING_DISABLED = 0x00, /**< The GATT proxy is running, but disabled. */
    CONFIG_GATT_PROXY_STATE_RUNNING_ENABLED  = 0x01, /**< The GATT proxy is running and enabled. */
    CONFIG_GATT_PROXY_STATE_UNSUPPORTED      = 0x02  /**< The GATT proxy feature is not supported. */
} config_gatt_proxy_state_t;
typedef struct __attribute((packed))
{
    uint8_t proxy_state; /**< The state of the GATT proxy service. */
} config_msg_proxy_status_t;
typedef struct __attribute((packed))
{
    uint8_t proxy_state; /**< The desired state of the GATT proxy service. */
} config_msg_proxy_set_t;
typedef enum
{
    CONFIG_FRIEND_STATE_SUPPORTED_DISABLED = 0x00, /**< Friendship is supported, but disabled. */
    CONFIG_FRIEND_STATE_SUPPORTED_ENABLED  = 0x01, /**< Friendship is supported and enabled. */
    CONFIG_FRIEND_STATE_UNSUPPORTED        = 0x02  /**< Friendship is not supported. */
} config_friend_state_t;
typedef struct __attribute((packed))
{
    uint8_t friend_state; /**< The desired state of the friendship feature. */
} config_msg_friend_set_t;
typedef struct __attribute((packed))
{
    uint8_t friend_state; /**< The state of the friendship feature. */
} config_msg_friend_status_t;
typedef struct __attribute((packed))
{
    config_msg_key_index_12_t netkey_index; /**< Index of the network to get the key refresh phase for. */
} config_msg_key_refresh_phase_get_t;
typedef struct __attribute((packed))
{
    config_msg_key_index_12_t netkey_index; /**< Index of the network to set the key refresh phase for. */
    uint8_t transition;                     /**< ID of the phase to transition to. */
} config_msg_key_refresh_phase_set_t;
typedef struct __attribute((packed))
{
    uint8_t status;                         /**< Status code. */
    config_msg_key_index_12_t netkey_index; /**< Index of the network the key refresh phase is reported for. */
    uint8_t phase;                          /**< Current key refresh phase for the subnet. */
} config_msg_key_refresh_phase_status_t;
typedef struct __attribute((packed))
{
    uint16_t destination;                   /**< Heartbeat publication destination. */
    uint8_t count_log;                      /**< Number of heartbeat messages to be sent. */
    uint8_t period_log;                     /**< Period of transmitted heartbeat messages. */
    uint8_t ttl;                            /**< TTL for heartbeat message. */
    uint16_t features;                      /**< Features triggering heartbeat messages. */
    config_msg_key_index_12_t netkey_index; /**< Index for the network key used to send heartbeats. */
} config_msg_heartbeat_publication_set_t;
typedef struct __attribute((packed))
{
    uint8_t status;                         /**< Status code. */
    uint16_t destination;                   /**< Heartbeat publication destination. */
    uint8_t count_log;                      /**< Number of heartbeat messages to be sent. */
    uint8_t period_log;                     /**< Period of transmitted heartbeat messages. */
    uint8_t ttl;                            /**< TTL for heartbeat message. */
    uint16_t features;                      /**< Features triggering heartbeat messages. */
    config_msg_key_index_12_t netkey_index; /**< Index for the network key used to send heartbeats. */
} config_msg_heartbeat_publication_status_t;
typedef struct __attribute((packed))
{
    uint16_t source;                        /**< Source of heartbeat messages. */
    uint16_t destination;                   /**< Destination of heartbeat messages. */
    uint8_t period_log;                     /**< Period of heartbeat messages. */
} config_msg_heartbeat_subscription_set_t;
typedef struct __attribute((packed))
{
    uint8_t status;                         /**< Status code. */
    uint16_t source;                        /**< Source of heartbeat messages. */
    uint16_t destination;                   /**< Destination of heartbeat messages. */
    uint8_t period_log;                     /**< Period of heartbeat messages. */
    uint8_t count_log;                      /**< Number of heartbeat messages received. */
    uint8_t min_hops;                       /**< Least number of hops in received heartbeat messages. */
    uint8_t max_hops;                       /**< Largest number of hops in received heartbeat messages. */
} config_msg_heartbeat_subscription_status_t;
typedef struct __attribute((packed))
{
    uint16_t element_address;               /**< Unicast address of the element. */
    config_msg_key_index_12_t appkey_index; /**< Application key index. */
    config_model_id_t model_id;             /**< Model ID. */
} config_msg_app_bind_unbind_t;
typedef struct __attribute((packed))
{
    uint8_t status;                         /**< Status code. */
    uint16_t element_address;               /**< Unicast address of the element. */
    config_msg_key_index_12_t appkey_index; /**< Application key index. */
    config_model_id_t model_id;             /**< Model ID. */
} config_msg_app_status_t;
typedef enum
{
    CONFIG_IDENTITY_STATE_STOPPED     = 0x00, /**< The node identity advertisement is stopped. */
    CONFIG_IDENTITY_STATE_RUNNING     = 0x01, /**< The node identity advertisement is running. */
    CONFIG_IDENTITY_STATE_UNSUPPORTED = 0x02  /**< Node identity advertising is not supported. */
} config_identity_state_t;
typedef struct __attribute((packed))
{
    config_msg_key_index_12_t netkey_index; /**< Subnet index. */
} config_msg_identity_get_t;

/** Message format for the Node Identity Set message. */
typedef struct __attribute((packed))
{
    config_msg_key_index_12_t netkey_index; /**< Subnet index. */
    uint8_t identity_state;                 /**< Current state of the node identity advertisement. */
} config_msg_identity_set_t;

/** Message format for the Node Identity Status message. */
typedef struct __attribute((packed))
{
    uint8_t status;                         /**< Status code. */
    config_msg_key_index_12_t netkey_index; /**< Subnet index. */
    uint8_t identity_state;                 /**< Identity advertisement state. */
} config_msg_identity_status_t;

/** Message format for the Composition Data Get message. */
typedef struct __attribute((packed))
{
    uint8_t page_number; /**< Page-number for the composition data page to retrieve. */
} config_msg_composition_data_get_t;

/** Message format for the Composition Data Status message. */
typedef struct __attribute((packed))
{
    uint8_t page_number; /**< Page-number for the composition data page contained in this response. */
    uint8_t data[];      /**< Composition data. */
} config_msg_composition_data_status_t;

/** Values for the relay state. */
typedef enum
{
    CONFIG_RELAY_STATE_SUPPORTED_DISABLED = 0x00, /**< Relaying is supported, but disabled. */
    CONFIG_RELAY_STATE_SUPPORTED_ENABLED  = 0x01, /**< Relaying is supported and enabled. */
    CONFIG_RELAY_STATE_UNSUPPORTED        = 0x02  /**< Relaying is not supported. */
} config_relay_state_t;

/** Message format for the Relay Status message. */
typedef struct __attribute((packed))
{
    uint8_t relay_state;                         /**< Current state of the relaying feature. */
    uint8_t relay_retransmit_count          : 3; /**< Number of retransmissions per relayed packet. */
    uint8_t relay_retransmit_interval_steps : 5; /**< Number of 10 ms steps between retransmissions. */
} config_msg_relay_status_t;

/** Message format for the Relay Set message. */
typedef struct __attribute((packed))
{
    uint8_t relay_state;                         /**< Desired state of the relaying feature. */
    uint8_t relay_retransmit_count          : 3; /**< Desired number of retransmissions per relayed packed. */
    uint8_t relay_retransmit_interval_steps : 5; /**< Desired number of 10 ms steps between retransmissions. */
} config_msg_relay_set_t;

/** Message format for the Network Transmit Set message. */
typedef struct __attribute((packed))
{
    uint8_t network_transmit_count          : 3; /**< Desired number of retransmissions per packed. */
    uint8_t network_transmit_interval_steps : 5; /**< Desired number of 10 ms steps between retransmissions. */
} config_msg_network_transmit_set_t;

/** Message format for the Network Transmit Status message. */
typedef struct __attribute((packed))
{
    uint8_t network_transmit_count          : 3; /**< Number of retransmissions per transmited packet. */
    uint8_t network_transmit_interval_steps : 5; /**< Number of 10 ms steps between retransmissions. */
} config_msg_network_transmit_status_t;

/** Message format for the SIG/Vendor Model App Get message. */
typedef struct __attribute((packed))
{
    uint16_t element_address;   /**< Unicast address of the element. */
    config_model_id_t model_id; /**< Model ID. */
} config_msg_model_app_get_t;

/** Message format for the SIG Model App List message. */
typedef struct __attribute((packed))
{
    uint8_t status;           /**< Status code. */
    uint16_t element_address; /**< Unicast address of the element. */
    uint16_t sig_model_id;    /**< SIG model ID. */
    uint8_t key_indexes[];    /**< Key indexes. */
} config_msg_sig_model_app_list_t;

/** Message format for the Vendor Model App List message. */
typedef struct __attribute((packed))
{
    uint8_t status;             /**< Status code. */
    uint16_t element_address;   /**< Unicast address of the element. */
    uint16_t vendor_company_id; /**< Vendor company ID. */
    uint16_t vendor_model_id;   /**< Vendor model ID. */
    uint8_t key_indexes[];      /**< List of application key indexes. */
} config_msg_vendor_model_app_list_t;

/** Message format for the SIG/Vendor Model Subscription Get message. */
typedef struct __attribute((packed))
{
    uint16_t element_address;   /**< Unicast address of the element. */
    config_model_id_t model_id; /**< Model ID. */
} config_msg_model_subscription_get_t;

/** Message format for the SIG Model Subscription List message. */
typedef struct __attribute((packed))
{
    uint8_t status;           /**< Status code. */
    uint16_t element_address; /**< Unicast address of the element. */
    uint16_t sig_model_id;    /**< SIG model ID. */
    uint16_t subscriptions[]; /**< Subscription list. */
} config_msg_sig_model_subscription_list_t;

/** Message format for the Vendor Model Subscription List message. */
typedef struct __attribute((packed))
{
    uint8_t status;             /**< Status code. */
    uint16_t element_address;   /**< Unicast address of the element. */
    uint16_t vendor_company_id; /**< Vendor company ID. */
    uint16_t vendor_model_id;   /**< Vendor model ID. */
    uint16_t subscriptions[];   /**< Subscription list. */
} config_msg_vendor_model_subscription_list_t;

/** Message format for the Low Power node PollTimeout Get message. */
typedef struct __attribute((packed))
{
    uint16_t lpn_address;       /**< The unicast address of the Low Power node. */
} config_msg_low_power_node_polltimeout_get_t;

/** Message format for the Low Power node PollTimeout Status message. */
typedef struct __attribute((packed))
{
    uint16_t lpn_address;       /**< The unicast address of the Low Power node. */
    uint8_t  polltimeout[3];    /**< Poll Timeout. */
} config_msg_low_power_node_polltimeout_status_t;



#endif

