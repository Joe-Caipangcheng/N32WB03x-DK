#ifndef BROADCAST_H__
#define BROADCAST_H__

#include <stdint.h>
#include "bearer_handler.h"
#include "packet.h"
#include "co_timer_schedule.h"

typedef struct broadcast_params broadcast_params_t;

typedef void (*broadcast_complete_cb_t) (broadcast_params_t * p_broadcast, timestamp_t timestamp);

struct broadcast_params
{
    packet_t * p_packet;
    uint32_t access_address;
    broadcast_complete_cb_t tx_complete_cb;
};

typedef struct
{
    bearer_action_t action; 
    broadcast_params_t params; 
    bool active; 
} broadcast_t;

uint32_t broadcast_send(broadcast_t * p_broadcast);
void broadcast_init(void);
void broadcast_stop(void);

#endif /* BROADCAST_H__ */
