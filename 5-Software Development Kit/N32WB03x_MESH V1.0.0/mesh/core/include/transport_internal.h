#ifndef TRANSPORT_INTERNAL_H__
#define TRANSPORT_INTERNAL_H__

#include <stdint.h>
#include <stdbool.h>
#include "transport.h"


#define TRANSPORT_SAR_SEQAUTH_IV_INDEX_OFFSET (NETWORK_SEQNUM_BITS)
#define TRANSPORT_SAR_SEQZERO_MASK (TRANSPORT_SAR_SEQNUM_DIFF_MAX)

typedef enum
{
    TRANSPORT_PACKET_RECEIVER_NONE      = 0,        /**< No one is receiving this packet. */
    TRANSPORT_PACKET_RECEIVER_SELF      = (1 << 0), /**< This node is receiving this packet. */
    TRANSPORT_PACKET_RECEIVER_FRIEND    = (1 << 1), /**< A friendship device is receiving this packet. */
} transport_packet_receiver_t;

typedef struct
{
    uint16_t seq_zero;
    uint8_t segment_offset;
    uint8_t last_segment;
} transport_segmentation_metadata_t;

typedef struct
{
    bool segmented;
    transport_packet_receiver_t receivers;
    uint8_t mic_size;
    union
    {
        struct
        {
            bool using_app_key;
            uint8_t app_key_id;
        } access;
        struct
        {
            transport_control_opcode_t opcode;
        } control;
    } type;
    transport_segmentation_metadata_t segmentation;
    network_packet_metadata_t net;
    const mesh_application_secmat_t * p_security_material;
    mesh_tx_token_t token;
    core_tx_bearer_selector_t tx_bearer_selector; /**< The bearer on which the outgoing packets are to be sent on. */
} transport_packet_metadata_t;


static inline uint32_t transport_sar_first_seq_num_get(uint32_t sequence_number, uint16_t seq_zero)
{
    if ((sequence_number & TRANSPORT_SAR_SEQZERO_MASK) < seq_zero)
    {
        return ((sequence_number - ((sequence_number & TRANSPORT_SAR_SEQZERO_MASK) - seq_zero) - (TRANSPORT_SAR_SEQZERO_MASK + 1)));
    }
    else
    {
        return ((sequence_number - ((sequence_number & TRANSPORT_SAR_SEQZERO_MASK) - seq_zero)));
    }
}


static inline uint64_t transport_sar_seqauth_get(uint32_t iv_index, uint32_t sequence_number, uint16_t seq_zero)
{
    return ((uint64_t) iv_index << TRANSPORT_SAR_SEQAUTH_IV_INDEX_OFFSET)
            + transport_sar_first_seq_num_get(sequence_number, seq_zero);
}


#endif /* TRANSPORT_INTERNAL_H__ */
