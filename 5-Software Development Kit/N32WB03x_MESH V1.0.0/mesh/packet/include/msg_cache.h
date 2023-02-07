#ifndef MSG_CACHE_H__
#define MSG_CACHE_H__

#include <stdbool.h>
#include <stdint.h>

#include "packet.h"
#include "mesh.h"
#include "mesh_config_core.h"


void msg_cache_init(void);
bool msg_cache_entry_exists(uint16_t src_addr, uint32_t sequence_number);
void msg_cache_entry_add(uint16_t src, uint32_t seq);
void msg_cache_clear(void);


#endif /* MSG_CACHE_H__ */

