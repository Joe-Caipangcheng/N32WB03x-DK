#ifndef CORE_TX_H__
#define CORE_TX_H__

#include <stdint.h>
#include <stdbool.h>
#include "mesh.h"
#include "net_packet.h"
#include "mesh_list.h"





typedef enum
{
    CORE_TX_ROLE_ORIGINATOR, 
    CORE_TX_ROLE_RELAY, 
    CORE_TX_ROLE_COUNT 
} core_tx_role_t;

typedef enum
{
    CORE_TX_BEARER_TYPE_INVALID     = 0x00,
    CORE_TX_BEARER_TYPE_ADV         = 0x01,
    CORE_TX_BEARER_TYPE_GATT_SERVER = 0x02,
    CORE_TX_BEARER_TYPE_GATT_CLIENT = 0x04,
    CORE_TX_BEARER_TYPE_FRIEND      = 0x08,
    CORE_TX_BEARER_TYPE_LOW_POWER   = 0x10,
    CORE_TX_BEARER_TYPE_LOCAL       = 0x20,
    CORE_TX_BEARER_TYPE_ALLOW_ALL   = 0xFF,
} core_tx_bearer_type_t;




typedef uint32_t core_tx_bearer_selector_t;

typedef struct
{
    core_tx_role_t role; 
    uint32_t net_packet_len; 
    const network_packet_metadata_t * p_metadata; 
    mesh_tx_token_t token; 
    core_tx_bearer_selector_t bearer_selector; 
} core_tx_alloc_params_t;




typedef void (*core_tx_complete_cb_t)(core_tx_role_t role,
                                      uint32_t bearer_index,
                                      uint32_t timestamp,
                                      mesh_tx_token_t token);


typedef uint32_t core_tx_bearer_bitmap_t;
typedef struct core_tx_bearer core_tx_bearer_t;

typedef enum
{
    CORE_TX_ALLOC_SUCCESS,
    CORE_TX_ALLOC_FAIL_REJECTED,
    CORE_TX_ALLOC_FAIL_NO_MEM
} core_tx_alloc_result_t;
typedef core_tx_alloc_result_t (*core_tx_bearer_packet_alloc_t)(core_tx_bearer_t * p_bearer,
                                                                const core_tx_alloc_params_t * p_params);
typedef void (*core_tx_bearer_packet_send_t)(core_tx_bearer_t * p_bearer,
                                             const uint8_t * p_packet,
                                             uint32_t packet_length);
typedef void (*core_tx_bearer_packet_discard_t)(core_tx_bearer_t * p_bearer);                                             
typedef struct
{
    core_tx_bearer_packet_alloc_t packet_alloc;
    core_tx_bearer_packet_send_t packet_send;
    core_tx_bearer_packet_discard_t packet_discard;
} core_tx_bearer_interface_t;
struct core_tx_bearer
{
    const core_tx_bearer_interface_t * p_interface;
    uint8_t bearer_index;
    core_tx_bearer_type_t type;
    list_node_t list_node;
};
typedef enum
{
    CORE_TX_TYPE_PROV,
    CORE_TX_TYPE_ADV,
    CORE_TX_TYPE_GATT_RELAY,
} core_tx_type_t;


void core_tx_complete_cb_set(core_tx_complete_cb_t tx_complete_callback);
core_tx_bearer_bitmap_t core_tx_packet_alloc(const core_tx_alloc_params_t * p_params, uint8_t ** pp_packet);
void core_tx_packet_send(void);
void core_tx_packet_discard(void);
core_tx_bearer_type_t core_tx_bearer_type_get(uint32_t bearer_index);
uint32_t core_tx_bearer_count_get(void);
void core_tx_bearer_add(core_tx_bearer_t * p_bearer,
                        const core_tx_bearer_interface_t * p_interface,
                        core_tx_bearer_type_t type);
void core_tx_complete(core_tx_bearer_t * p_bearer,
                      core_tx_role_t role,
                      uint32_t timestamp,
                      mesh_tx_token_t token);
void core_tx_type_set(uint8_t type);
#endif /* CORE_TX_H__ */
