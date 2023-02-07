#ifndef MESH_CONFIG_BACKEND_H__
#define MESH_CONFIG_BACKEND_H__


#include "mesh_config.h"
#include "mesh_config_entry.h"
typedef enum
{
    MESH_CONFIG_BACKEND_EVT_TYPE_STORE_COMPLETE, 
    MESH_CONFIG_BACKEND_EVT_TYPE_ERASE_COMPLETE, 
    MESH_CONFIG_BACKEND_EVT_TYPE_STORAGE_MEDIUM_FAILURE, 
} mesh_config_backend_evt_type_t;

typedef struct
{
    mesh_config_backend_evt_type_t type;
    mesh_config_entry_id_t id;
} mesh_config_backend_evt_t;

typedef void (* mesh_config_backend_evt_cb_t)(const mesh_config_backend_evt_t * p_evt);
typedef enum
{
    MESH_CONFIG_BACKEND_ITERATE_ACTION_STOP, 
    MESH_CONFIG_BACKEND_ITERATE_ACTION_CONTINUE, 
} mesh_config_backend_iterate_action_t;
typedef mesh_config_backend_iterate_action_t (*mesh_config_backend_iterate_cb_t)(mesh_config_entry_id_t id, const uint8_t * p_entry, uint32_t entry_len);
void mesh_config_backend_init(const mesh_config_entry_params_t * p_entries,
                              uint32_t entry_count,
                              const mesh_config_file_params_t * p_files,
                              uint32_t file_count,
                              mesh_config_backend_evt_cb_t evt_cb);
uint32_t mesh_config_backend_store(mesh_config_entry_id_t id, const uint8_t * p_entry, uint32_t entry_len);
uint32_t mesh_config_backend_erase(mesh_config_entry_id_t id);
uint32_t mesh_config_backend_read(mesh_config_entry_id_t id, uint8_t * p_entry, uint32_t * p_entry_len);
void mesh_config_backend_read_all(mesh_config_backend_iterate_cb_t cb);
uint32_t mesh_config_backend_power_down_time_get(void);


#endif /* MESH_CONFIG_BACKEND_H__ */
