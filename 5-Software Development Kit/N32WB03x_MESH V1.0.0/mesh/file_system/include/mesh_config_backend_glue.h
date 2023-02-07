#ifndef MESH_CONFIG_BACKEND_GLUE_H_
#define MESH_CONFIG_BACKEND_GLUE_H_

#include <stdlib.h>
#include "mesh_config_backend_file.h"
#include "mesh_config_backend.h"
#include "mesh_config.h"

typedef struct
{
    const uint32_t * p_start; 
    uint32_t length;          
} mesh_config_backend_flash_usage_t;
void mesh_config_backend_glue_init(mesh_config_backend_evt_cb_t evt_cb);
uint32_t mesh_config_backend_file_create(mesh_config_backend_file_t * p_file);
uint32_t mesh_config_backend_record_write(mesh_config_backend_file_t * p_file, const uint8_t * p_data, uint32_t length);
uint32_t mesh_config_backend_record_erase(mesh_config_backend_file_t * p_file);
uint32_t mesh_config_backend_record_read(mesh_config_backend_file_t * p_file, uint8_t * p_data, uint32_t * p_length);
void mesh_config_backend_records_read(mesh_config_backend_file_t * p_file,
                                      mesh_config_backend_iterate_cb_t callback);
uint16_t mesh_config_record_size_calculate(uint16_t entry_size);
void mesh_config_backend_flash_usage_get(mesh_config_backend_flash_usage_t * p_usage);


#endif /* MESH_CONFIG_BACKEND_GLUE_H_ */
