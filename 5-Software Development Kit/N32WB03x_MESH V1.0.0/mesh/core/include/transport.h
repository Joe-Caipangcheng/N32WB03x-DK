#ifndef MESH_TRANSPORT_H__
#define MESH_TRANSPORT_H__

#include <stdint.h>
#include <stdbool.h>
#include "packet.h"
#include "utils.h"
#include "core_tx.h"
#include "mesh.h"
#include "mesh_defines.h"
#include "network.h"
#include "packet_mesh.h"
#include "utils.h"
#include "mesh_config_core.h"




#define TRANSPORT_SAR_RX_TIMEOUT_DEFAULT_US SEC_TO_US(10)
#define TRANSPORT_SAR_RX_ACK_BASE_TIMEOUT_DEFAULT_US MS_TO_US(150)
#define TRANSPORT_SAR_RX_ACK_PER_HOP_ADDITION_DEFAULT_US MS_TO_US(50)
#define TRANSPORT_SAR_TX_RETRY_BASE_TIMEOUT_DEFAULT_US MS_TO_US(500)
#define TRANSPORT_SAR_TX_RETRY_PER_HOP_ADDITION_DEFAULT_US MS_TO_US(50)
#define TRANSPORT_SAR_TX_RETRIES_DEFAULT (4)
#define TRANSPORT_CONTROL_PACKET_CONSUMERS_MAX   (1)
#define TRANSPORT_CONTROL_PACKET_OPCODE_MAX (PACKET_MESH_TRS_CONTROL_OPCODE_MASK)




typedef enum
{
    TRANSPORT_CONTROL_OPCODE_SEGACK,
    TRANSPORT_CONTROL_OPCODE_FRIEND_POLL,
    TRANSPORT_CONTROL_OPCODE_FRIEND_UPDATE,
    TRANSPORT_CONTROL_OPCODE_FRIEND_REQUEST,
    TRANSPORT_CONTROL_OPCODE_FRIEND_OFFER,
    TRANSPORT_CONTROL_OPCODE_FRIEND_CLEAR,
    TRANSPORT_CONTROL_OPCODE_FRIEND_CLEAR_CONFIRM,
    TRANSPORT_CONTROL_OPCODE_FRIEND_SUBSCRIPTION_LIST_ADD,
    TRANSPORT_CONTROL_OPCODE_FRIEND_SUBSCRIPTION_LIST_REMOVE,
    TRANSPORT_CONTROL_OPCODE_FRIEND_SUBSCRIPTION_LIST_CONFIRM,
    TRANSPORT_CONTROL_OPCODE_HEARTBEAT
} transport_control_opcode_t;




typedef struct
{
    transport_control_opcode_t opcode; 
    const packet_mesh_trs_control_packet_t * p_data; 
    uint32_t data_len; 
    bool reliable;
    uint16_t src;
    mesh_address_t dst; 
    const mesh_network_secmat_t * p_net_secmat;
    uint8_t ttl; 
    core_tx_bearer_selector_t bearer_selector; 
} transport_control_packet_t;


typedef void (*transport_control_packet_callback_t)(
    const transport_control_packet_t * p_control_packet,
    const mesh_rx_metadata_t * p_rx_metadata);

#define TRANSPORT_CONTROL_PACKET_OPCODE_MAX (PACKET_MESH_TRS_CONTROL_OPCODE_MASK)

typedef struct
{
    transport_control_opcode_t opcode; /**< Opcode to handle. */
    transport_control_packet_callback_t callback; /**< Callback function to call when a control packet with the given opcode is received. */
} transport_control_packet_handler_t;

void transport_init(void);
void transport_enable(void);
uint32_t transport_packet_in(const packet_mesh_trs_packet_t * p_packet,
                             uint32_t trs_packet_len,
                             const network_packet_metadata_t * p_net_metadata,
                             const mesh_rx_metadata_t * p_rx_metadata);

uint32_t transport_tx(const mesh_tx_params_t * p_params, uint32_t * const p_packet_reference);
uint32_t transport_control_tx(const transport_control_packet_t * p_params, mesh_tx_token_t tx_token);



#endif
