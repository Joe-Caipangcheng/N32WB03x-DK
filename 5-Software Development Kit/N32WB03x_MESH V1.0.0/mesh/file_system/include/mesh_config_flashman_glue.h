#ifndef MESH_CONFIG_FLASHMAN_GLUE_H_
#define MESH_CONFIG_FLASHMAN_GLUE_H_

#include "flash_manager.h"

typedef struct
{
    flash_manager_t flash_manager;
} mesh_config_backend_glue_data_t;

typedef struct
{
    fm_entry_t * p_entry;
} mesh_config_backend_glue_iterator_t;

#endif /* MESH_CONFIG_FLASHMAN_GLUE_H_ */
