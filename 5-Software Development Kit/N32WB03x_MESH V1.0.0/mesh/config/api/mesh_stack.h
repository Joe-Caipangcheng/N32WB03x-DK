#ifndef MESH_STACK_H__
#define MESH_STACK_H__

#include <stdint.h>
#include <stdbool.h>
#include "mesh_error.h"
#include "mesh.h"
#include "device_state_manager.h"
#include "mesh_prov_types.h"








uint32_t mesh_stack_provisioning_data_store(const mesh_prov_provisioning_data_t * p_prov_data,const uint8_t * p_devkey);




#endif /* MESH_STACK_H__ */
