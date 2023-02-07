#ifndef PROXY_H__
#define PROXY_H__

#include <stdint.h>
#include <stdbool.h>
#include "net_packet.h"
#include "mesh_config_core.h"
#include "packet_buffer.h"
#include "mesh_gatt_srv.h"
#include "mesh.h"





#ifndef PROXY_ENABLED_DEFAULT
#define PROXY_ENABLED_DEFAULT (true)
#endif


void proxy_init(void);
uint32_t proxy_enable(void);
uint32_t proxy_start(void);
uint32_t proxy_stop(void);
void proxy_disable(void);
uint32_t proxy_node_id_enable(const mesh_beacon_info_t * p_beacon_info, mesh_key_refresh_phase_t kr_phase);
uint32_t proxy_node_id_disable(void);
bool proxy_node_id_is_enabled(const mesh_beacon_info_t * p_beacon_info);
bool proxy_is_enabled(void);
bool proxy_is_connected(void);
void proxy_disconnect(void);
uint32_t proxy_stop(void);
uint32_t proxy_start(void);

#endif /* PROXY_H__ */
