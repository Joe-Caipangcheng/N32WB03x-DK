#ifndef MESH_CONFIGURE_H__
#define MESH_CONFIGURE_H__

#include <stdint.h>

void mesh_configure_device_uuid_set(const uint8_t* p_uuid);
void mesh_configure_device_uuid_reset(void);
const uint8_t* mesh_configure_device_uuid_get(void);

#endif /* MESH_CONFIGURE_H__ */
