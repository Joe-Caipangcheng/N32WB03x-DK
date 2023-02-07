#ifndef MESH_CONFIG_H__
#define MESH_CONFIG_H__

#include <stdint.h>
#include <stdbool.h>


typedef enum
{
    MESH_CONFIG_LOAD_FAILURE_INVALID_LENGTH, 
    MESH_CONFIG_LOAD_FAILURE_INVALID_DATA, 
    MESH_CONFIG_LOAD_FAILURE_INVALID_ID 
} mesh_config_load_failure_t;


void mesh_config_init(void);
void mesh_config_load(void);
void mesh_config_clear(void);
void mesh_config_power_down(void);
bool mesh_config_is_busy(void);
uint32_t mesh_config_power_down_time_get(void);

#endif /* MESH_CONFIG_H__ */
