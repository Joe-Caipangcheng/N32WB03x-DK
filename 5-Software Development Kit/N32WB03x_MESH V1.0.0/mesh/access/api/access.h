#ifndef ACCESS_H__
#define ACCESS_H__

#include <stdint.h>
#include "device_state_manager.h"
#include "mesh.h"


#define ACCESS_HANDLE_INVALID (0xFFFF)
#define ACCESS_COMPANY_ID_NONE (0xFFFF)
#define ACCESS_COMPANY_ID_NORDIC (0x0059)
#define ACCESS_ELEMENT_INDEX_INVALID (0xFFFF)
#define ACCESS_OPCODE_SIG(opcode)             { (opcode), ACCESS_COMPANY_ID_NONE }
#define ACCESS_OPCODE_VENDOR(opcode, company) { (opcode), (company) }
#define ACCESS_MODEL_SIG(id)  {.company_id = ACCESS_COMPANY_ID_NONE, .model_id = (id)}
#define ACCESS_MODEL_VENDOR(id, company) {.company_id = (company), .model_id = (id)}
#define ACCESS_TTL_USE_DEFAULT  (0xFF)
#define ACCESS_MESSAGE_LENGTH_MAX (MESH_SEG_PAYLOAD_SIZE_MAX)
#define ACCESS_PUBLISH_PERIOD_STEP_MAX (0x3F)
#define ACCESS_PUBLISH_STEP_RES_BITS (2)
#define ACCESS_PUBLISH_STEP_NUM_BITS (6)
#define ACCESS_PUBLISH_RETRANSMIT_COUNT_BITS (3)
#define ACCESS_PUBLISH_RETRANSMIT_INTERVAL_STEPS_BITS (5)
#define ACCESS_PUBLISH_PERIOD_NONE   { ACCESS_PUBLISH_RESOLUTION_100MS, 0 }

typedef struct __attribute((packed))
{
    uint16_t company_id;
    uint16_t model_id;
} access_model_id_t;


typedef uint16_t access_model_handle_t;
typedef void (*access_publish_timeout_cb_t)(access_model_handle_t handle, void * p_args);

typedef struct
{
    uint16_t opcode;
    uint16_t company_id;
} access_opcode_t;

typedef struct
{
    mesh_address_t src;
    mesh_address_t dst;
    uint8_t ttl;
    dsm_handle_t appkey_handle;
    const mesh_rx_metadata_t * p_core_metadata;
    dsm_handle_t subnet_handle;
} access_message_rx_meta_t;

typedef struct
{
    access_opcode_t opcode;
    const uint8_t * p_data;
    uint16_t length;
    access_message_rx_meta_t meta_data;
} access_message_rx_t;

typedef struct
{
    access_opcode_t opcode;
    const uint8_t * p_buffer;
    uint16_t length;
    bool force_segmented;
    mesh_transmic_size_t transmic_size;
    mesh_tx_token_t access_token;
} access_message_tx_t;

typedef void (*access_opcode_handler_cb_t)(access_model_handle_t handle,
                                           const access_message_rx_t * p_message,
                                           void * p_args);

typedef struct
{
    access_opcode_t opcode;
    access_opcode_handler_cb_t handler;
} access_opcode_handler_t;

typedef struct
{
    access_model_id_t model_id;
    uint16_t element_index;
    const access_opcode_handler_t * p_opcode_handlers;
    uint32_t opcode_count;
    void * p_args;
    access_publish_timeout_cb_t publish_timeout_cb;
} access_model_add_params_t;


typedef struct
{
    uint8_t step_res : ACCESS_PUBLISH_STEP_RES_BITS;
    uint8_t step_num : ACCESS_PUBLISH_STEP_NUM_BITS;
} access_publish_period_t;

typedef struct
{
    uint8_t count : ACCESS_PUBLISH_RETRANSMIT_COUNT_BITS;
    uint8_t interval_steps : ACCESS_PUBLISH_RETRANSMIT_INTERVAL_STEPS_BITS;
} access_publish_retransmit_t;

typedef enum
{
    ACCESS_PUBLISH_RESOLUTION_100MS = 0,
    ACCESS_PUBLISH_RESOLUTION_1S    = 1,
    ACCESS_PUBLISH_RESOLUTION_10S   = 2,
    ACCESS_PUBLISH_RESOLUTION_10MIN = 3,
    ACCESS_PUBLISH_RESOLUTION_MAX = ACCESS_PUBLISH_RESOLUTION_10MIN
} access_publish_resolution_t;


void access_init(void);
uint32_t access_model_add(const access_model_add_params_t * p_model_params, access_model_handle_t * p_model_handle);
void access_incoming_handle(const access_message_rx_t * p_message);
uint32_t access_model_reply(access_model_handle_t handle,
                            const access_message_rx_t * p_message,
                            const access_message_tx_t * p_reply);
uint32_t access_model_p_args_get(access_model_handle_t handle, void ** pp_args);
                            
static inline void dsm_local_unicast_addresses_get(dsm_local_unicast_address_t * p_address)
{
    memcpy(p_address, &dsm_local_unicast_addr, sizeof(dsm_local_unicast_address_t));
}                            
                            
                            
static inline bool is_element_rx_address(const mesh_address_t * p_dst, uint16_t * p_index)
{
    if (p_dst->type == MESH_ADDRESS_TYPE_UNICAST)
    {
        dsm_local_unicast_address_t local_addresses;
        dsm_local_unicast_addresses_get(&local_addresses);

        *p_index = p_dst->value - local_addresses.address_start;
        return true;
    }
    else if (p_dst->value == MESH_ALL_PROXIES_ADDR ||
             p_dst->value == MESH_ALL_FRIENDS_ADDR ||
             p_dst->value == MESH_ALL_RELAYS_ADDR ||
             p_dst->value == MESH_ALL_NODES_ADDR)
    {
        *p_index = 0;
        return true;
    }
    return false;
}                            
                            
uint32_t access_model_publish(access_model_handle_t handle, const access_message_tx_t * p_message);
uint32_t access_packet_tx(access_model_handle_t handle,
                          const access_message_tx_t * p_tx_message,
                          const uint8_t *p_access_payload,
                          uint16_t access_payload_len);

uint32_t access_model_publication_by_appkey_stop(dsm_handle_t appkey_handle);

#endif /* ACCESS_H__ */
