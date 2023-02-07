#ifndef MESH_HEARTBEAT_H__
#define MESH_HEARTBEAT_H__

#include <stdint.h>
#include <stdbool.h>
#include "packet.h"
#include "utils.h"
#include "mesh.h"
#include "transport.h"

#define HEARTBEAT_MAX_PERIOD_LOG (0x11)
#define HEARTBEAT_MAX_PERIOD     (1u << (HEARTBEAT_MAX_PERIOD_LOG - 1))
#define HEARTBEAT_INVALID_PERIOD (0xFFFFFFFF)

#define HEARTBEAT_MAX_COUNT_LOG  (0x10)
#define HEARTBEAT_MAX_COUNT      (1u << (HEARTBEAT_MAX_COUNT_LOG - 1))
#define HEARTBEAT_INF_COUNT      (0xFFFF)
#define HEARTBEAT_INF_COUNT_LOG  (0xFF)
#define HEARTBEAT_INVALID_COUNT  (0xFFFFFFFF)

typedef struct
{
    uint16_t            dst;
    uint32_t            count;
    uint32_t            period;
    uint8_t             ttl;
    uint16_t           features;
    uint16_t    netkey_index;
} heartbeat_publication_state_t;

typedef struct
{
    uint16_t    local_address;
    const mesh_network_secmat_t * p_net_secmat;
} heartbeat_publication_information_t;

typedef struct
{
    uint16_t  src;
    uint16_t  dst;
    uint32_t            count;
    uint32_t            period;
    uint16_t            min_hops;
    uint16_t            max_hops;

} heartbeat_subscription_state_t;

typedef uint32_t (* hb_pub_info_getter_t)(heartbeat_publication_information_t * p_pub_info);

void heartbeat_public_info_getter_register(hb_pub_info_getter_t p_getter);
void heartbeat_init(void);
const heartbeat_publication_state_t * heartbeat_publication_get(void);
uint32_t heartbeat_publication_set(const heartbeat_publication_state_t * p_publication_state);
uint32_t heartbeat_subscription_set(const heartbeat_subscription_state_t * p_hb_sub);
const heartbeat_subscription_state_t * heartbeat_subscription_get(void);

#endif
