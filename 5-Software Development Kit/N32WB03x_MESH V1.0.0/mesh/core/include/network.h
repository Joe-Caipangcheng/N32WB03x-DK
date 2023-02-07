#ifndef MESH_NETWORK_H__
#define MESH_NETWORK_H__

#include "mesh.h"
#include "net_packet.h"
#include "core_tx.h"




typedef struct
{
    struct
    {
        network_packet_metadata_t * p_metadata;
        mesh_tx_token_t token;
        uint32_t payload_len;
        core_tx_bearer_selector_t bearer_selector;
        core_tx_role_t role;
    } user_data;
    uint8_t * p_payload;
} network_tx_packet_buffer_t;




uint32_t network_packet_in(const uint8_t * p_packet, uint32_t net_packet_len, const mesh_rx_metadata_t * p_rx_metadata);
uint32_t network_packet_alloc(network_tx_packet_buffer_t * p_buffer);
void network_packet_send(const network_tx_packet_buffer_t * p_buffer);
void network_init(const mesh_init_params_t * p_init_params);
void network_enable(void);


#endif //MESH_NETWORK_H__
