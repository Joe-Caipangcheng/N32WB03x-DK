#ifndef ADVERTISER_H__
#define ADVERTISER_H__

#include "packet_buffer.h"
#include "mesh_defines.h"
#include "mesh.h"
#include "bearer_event.h"
#include "broadcast.h"
#include "stdio.h"

#define ADVERTISER_INTERVAL_RANDOMIZATION_US MS_TO_US(10)
#define ADVERTISER_REPEAT_INFINITE 0xFF
#define ADVERTISER_PACKET_BUFFER_PACKET_MAXLEN    (sizeof(packet_buffer_packet_t) + sizeof(adv_packet_t))

typedef struct
{
    mesh_tx_token_t token;
    struct
    {
        uint8_t repeats;
    } config;
    packet_t packet __attribute__((aligned(WORD_SIZE)));
} adv_packet_t;


typedef struct
{
    ble_gap_addr_t        adv_addr;
    uint32_t              advertisement_interval_us;
} advertiser_config_t;

typedef struct advertiser_t advertiser_t;
typedef void (*advertiser_tx_complete_cb_t)(advertiser_t * p_adv, mesh_tx_token_t token, timestamp_t timestamp);

typedef struct
{
    mesh_tx_token_t token; 
    timestamp_t timestamp; 
} advertiser_tx_complete_params_t;

struct advertiser_t
{
    bool                            enabled; 
    adv_packet_t *                  p_packet; 
    broadcast_t                     broadcast; 
    timer_event_t                   timer; 
    packet_buffer_t                 buf; 
    advertiser_config_t             config; 
    advertiser_tx_complete_cb_t     tx_complete_callback; 
    bearer_event_sequential_t       tx_complete_event; 
    advertiser_tx_complete_params_t tx_complete_params; 
};

typedef struct advertiser_t advertiser_t;

void advertiser_init(void);
void advertiser_instance_init(advertiser_t * p_adv,
                              advertiser_tx_complete_cb_t tx_complete_cb,
                              uint8_t * p_buffer,
                              uint32_t buffer_size);
void advertiser_enable(advertiser_t * p_adv);
void advertiser_disable(advertiser_t * p_adv);
adv_packet_t * advertiser_packet_alloc(advertiser_t * p_adv, uint32_t adv_payload_size);
void advertiser_packet_send(advertiser_t * p_adv, adv_packet_t * p_packet);
void advertiser_packet_discard(advertiser_t * p_adv, adv_packet_t * p_packet);
static inline bool advertiser_is_enabled(const advertiser_t * p_adv)
{
    return p_adv->enabled;
}

#endif /* ADVERTISER_H__ */
