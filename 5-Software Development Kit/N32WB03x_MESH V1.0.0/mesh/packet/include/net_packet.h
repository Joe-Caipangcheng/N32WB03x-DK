#ifndef NET_PACKET_H__
#define NET_PACKET_H__

#include <stdint.h>
#include <stdbool.h>
#include "packet_mesh.h"
#include "mesh.h"



#define NET_PACKET_ENCRYPTION_START_OFFSET PACKET_MESH_NET_DST0_OFFSET
#define NET_PACKET_OBFUSCATION_START_OFFSET PACKET_MESH_NET_CTL_OFFSET
#define NET_PACKET_ENCRYPTION_START_PAYLOAD_OVERHEAD (PACKET_MESH_NET_PDU_OFFSET - NET_PACKET_ENCRYPTION_START_OFFSET)





typedef enum
{
    NET_PACKET_KIND_TRANSPORT, 
    NET_PACKET_KIND_PROXY_CONFIG,
} net_packet_kind_t;

typedef struct
{
    mesh_address_t dst;
    uint16_t src;
    uint8_t ttl;
    bool control_packet;
    struct
    {
        uint32_t sequence_number;
        uint32_t iv_index;
    } internal;
    const mesh_network_secmat_t * p_security_material;
} network_packet_metadata_t;

uint32_t net_packet_decrypt(network_packet_metadata_t * p_net_metadata,
                        uint32_t net_packet_len,
                        const packet_mesh_net_packet_t * p_net_encrypted_packet,
                        packet_mesh_net_packet_t * p_net_decrypted_packet,
                        net_packet_kind_t packet_kind);
                        
void net_packet_encrypt(network_packet_metadata_t * p_net_metadata,
                        uint32_t payload_len,
                        packet_mesh_net_packet_t * p_net_packet,
                        net_packet_kind_t packet_kind);

void net_packet_header_set(packet_mesh_net_packet_t * p_net_packet,
                           const network_packet_metadata_t * p_metadata);

uint32_t net_packet_payload_len_get(const network_packet_metadata_t * p_net_metadata,
                                    uint32_t net_packet_len);

static inline uint32_t net_packet_mic_size_get(bool is_control_packet)
{
    return (is_control_packet ? 8 : 4);
}
static inline uint8_t * net_packet_obfuscation_start_get(const packet_mesh_net_packet_t * p_net_packet)
{
    return ((uint8_t *) p_net_packet) + NET_PACKET_OBFUSCATION_START_OFFSET;
}
static inline packet_mesh_net_packet_t * net_packet_from_payload(const uint8_t * p_net_payload)
{
    return (packet_mesh_net_packet_t *) (p_net_payload - PACKET_MESH_NET_PDU_OFFSET);
}

static inline uint8_t * net_packet_enc_start_get(const packet_mesh_net_packet_t * p_net_packet)
{
    return ((uint8_t *) p_net_packet) + NET_PACKET_ENCRYPTION_START_OFFSET;
}

static inline uint8_t * net_packet_payload_get(const packet_mesh_net_packet_t * p_net_packet)
{
    return (uint8_t *) &p_net_packet->pdu[PACKET_MESH_NET_PDU_OFFSET];
}
static inline uint32_t net_packet_payload_len_get(const network_packet_metadata_t * p_net_metadata,
                                    uint32_t net_packet_len)
{
    return net_packet_len - PACKET_MESH_NET_PDU_OFFSET -
           net_packet_mic_size_get(p_net_metadata->control_packet);
}

#endif /* NET_PACKET_H__ */

