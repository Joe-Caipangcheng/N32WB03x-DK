#ifndef MESH_CONFIG_BACKEND_FILE_H_
#define MESH_CONFIG_BACKEND_FILE_H_

#include <stdlib.h>
#include "mesh_config_core.h"
#include "mesh_config_flashman_glue.h"


typedef struct
{
    mesh_config_backend_glue_iterator_t iterator;
} mesh_config_backend_record_iterator_t;

typedef struct
{
    uint16_t                        file_id;        
    uint16_t                        size;           
    uint16_t                        entry_count;    
    uint16_t                        curr_pos;       
    mesh_config_backend_glue_data_t glue_data;      
} mesh_config_backend_file_t;

#endif /* MESH_CONFIG_BACKEND_FILE_H_ */
