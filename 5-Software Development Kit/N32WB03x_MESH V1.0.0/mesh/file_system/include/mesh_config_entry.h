#ifndef MESH_CONFIG_ENTRY_H__
#define MESH_CONFIG_ENTRY_H__

#include <stdint.h>
#include <stdbool.h>
#include "mesh_section.h"
#include "mesh_error.h"
#include "mesh_common.h"
#include "mesh_config_backend_file.h"

#define MESH_CONFIG_ENTRY_ID(FILE, RECORD) (const mesh_config_entry_id_t) {(FILE), (RECORD)}
#define MESH_CONFIG_ENTRY_MAX_SIZE 64
#define MESH_CONFIG_FILE(NAME, FILE_ID, STRATEGY)                                                   \
    static mesh_config_backend_file_t CONCAT_2(NAME, _backend_data);                                \
    MESH_SECTION_ITEM_REGISTER_FLASH(mesh_config_files, const mesh_config_file_params_t NAME) = \
        {.id = FILE_ID, .strategy = STRATEGY, .p_backend_data = &CONCAT_2(NAME, _backend_data)}

#define MESH_CONFIG_ENTRY(NAME, ID, MAX_COUNT, ENTRY_SIZE, SET_CB, GET_CB, DELETE_CB, HAS_DEFAULT_VALUE)            \
    static mesh_config_entry_flags_t m_##NAME##_state[MAX_COUNT];                                                   \
    MESH_SECTION_ITEM_REGISTER_FLASH(mesh_config_entries,                                                       \
                                         const mesh_config_entry_params_t m_##NAME##_params) =                      \
        {.p_id        = &(ID),                                                                                      \
         .entry_size  = ENTRY_SIZE,                                                                                 \
         .has_default = HAS_DEFAULT_VALUE,                                                                          \
         .max_count   = MAX_COUNT,                                                                                  \
         .callbacks   = {SET_CB, GET_CB, DELETE_CB},                                                                \
         .p_state     = m_##NAME##_state}

#define MESH_CONFIG_ENTRY_API_DEFINE(NAME, ID, DATA_TYPE)       \
    uint32_t NAME##_set(const DATA_TYPE * p_entry)              \
    {                                                           \
        return mesh_config_entry_set((ID), p_entry);            \
    }                                                           \
    uint32_t NAME##_get(DATA_TYPE * p_entry)                    \
    {                                                           \
        return mesh_config_entry_get((ID), p_entry);            \
    }                                                           \
    uint32_t NAME##_delete(void)                                \
    {                                                           \
        return mesh_config_entry_delete((ID));                  \
    }

#define MESH_CONFIG_ENTRY_ARRAY_WRAPPER_DECLARE(NAME, ID, DATA_TYPE, INDEX_TYPE, MAX_COUNT) \
    uint32_t NAME##_set(INDEX_TYPE index, const DATA_TYPE * p_entry)    \
    {                                                                   \
        if (index >= (MAX_COUNT)) return MESH_ERROR_INVALID_PARAM;       \
        mesh_config_entry_id_t id = ID;                                 \
        id.record += (uint16_t) index;                                  \
        return mesh_config_entry_set(id, p_entry);                      \
    }                                                                   \
    uint32_t NAME##_get(INDEX_TYPE index, DATA_TYPE * p_entry)          \
    {                                                                   \
        if (index >= (MAX_COUNT)) return MESH_ERROR_INVALID_PARAM;       \
        mesh_config_entry_id_t id = ID;                                 \
        id.record += (uint16_t) index;                                  \
        return mesh_config_entry_get(id, p_entry);                      \
    }                                                                   \
    uint32_t NAME##_delete(INDEX_TYPE index)                            \
    {                                                                   \
        if (index >= (MAX_COUNT)) return MESH_ERROR_INVALID_PARAM;       \
        mesh_config_entry_id_t id = ID;                                 \
        id.record += (uint16_t) index;                                  \
        return mesh_config_entry_delete(id);                            \
    }

typedef struct
{
    uint16_t file;
    uint16_t record;
} mesh_config_entry_id_t;

typedef enum
{
    MESH_CONFIG_STRATEGY_NON_PERSISTENT, 
    MESH_CONFIG_STRATEGY_CONTINUOUS,     
    MESH_CONFIG_STRATEGY_ON_POWER_DOWN,  
} mesh_config_strategy_t;

typedef uint32_t (*mesh_config_entry_set_t)(mesh_config_entry_id_t id, const void * p_entry);
typedef void (*mesh_config_entry_get_t)(mesh_config_entry_id_t id, void * p_entry);
typedef void (*mesh_config_entry_delete_t)(mesh_config_entry_id_t id);

typedef struct
{
    uint16_t id; 
    mesh_config_strategy_t strategy; 
    mesh_config_backend_file_t * p_backend_data; 
} mesh_config_file_params_t;

typedef enum
{
    MESH_CONFIG_ENTRY_FLAG_DIRTY  = (1 << 0), 
    MESH_CONFIG_ENTRY_FLAG_ACTIVE = (1 << 1), 
    MESH_CONFIG_ENTRY_FLAG_BUSY   = (1 << 2), 
} mesh_config_entry_flags_t;
typedef struct
{
    const mesh_config_entry_id_t * p_id;
    uint16_t entry_size; 
    uint16_t max_count; 
    bool has_default;
    struct
    {
        mesh_config_entry_set_t setter;
        mesh_config_entry_get_t getter;
        mesh_config_entry_delete_t deleter;
    } callbacks;
    mesh_config_entry_flags_t * p_state; 
} mesh_config_entry_params_t;
bool mesh_config_entry_available_id(mesh_config_entry_id_t * p_id);
uint32_t mesh_config_entry_set(mesh_config_entry_id_t id, const void * p_entry);
uint32_t mesh_config_entry_get(mesh_config_entry_id_t id, void * p_entry);
uint32_t mesh_config_entry_delete(mesh_config_entry_id_t id);


#endif /* MESH_CONFIG_ENTRY_H__ */
