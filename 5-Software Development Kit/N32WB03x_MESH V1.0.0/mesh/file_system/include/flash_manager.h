#ifndef FLASH_MANAGER_H__
#define FLASH_MANAGER_H__

#include <stdint.h>
#include <stdbool.h>
#include "mesh_queue.h"
#include "toolchain.h"
#include "bearer_event.h"
#include "utils.h"




#define FLASH_MANAGER_DATA_PER_PAGE (PAGE_SIZE - sizeof(flash_manager_metadata_t))
#define FLASH_MANAGER_PAGE_COUNT_MINIMUM(DATA_SIZE, LARGEST_ENTRY_SIZE) (1 + ((DATA_SIZE) / (FLASH_MANAGER_DATA_PER_PAGE - LARGEST_ENTRY_SIZE)))
#define FLASH_MANAGER_HANDLE_MAX        (0x7EFF) 
#define FLASH_MANAGER_HANDLE_INVALID    (0x0000) 

#define FLASH_MANAGER_ENTRY_LEN_OVERHEAD (sizeof(fm_header_t) / WORD_SIZE) 
typedef uint16_t fm_handle_t;

typedef struct
{
    uint16_t len_words; 
    fm_handle_t handle; 
} fm_header_t;
typedef struct
{
    fm_handle_t mask;  
    fm_handle_t match; 
} fm_handle_filter_t;
typedef struct
{
    fm_header_t header;
    uint32_t data[];
} fm_entry_t;
typedef enum
{
    FM_STATE_UNINITIALIZED,
    FM_STATE_BUILDING,
    FM_STATE_READY,
    FM_STATE_DEFRAG,
    FM_STATE_REMOVING
} fm_state_t;
typedef struct
{
    uint8_t metadata_len; 
    uint8_t entry_header_length; 
    uint8_t entry_len_length_bits;  
    uint8_t entry_type_length_bits; 

    uint8_t pages_in_area; 
    uint8_t page_index;   
    uint16_t _padding; 
} flash_manager_metadata_t;

typedef union
{
    flash_manager_metadata_t metadata; 
    uint8_t raw[PAGE_SIZE];
} flash_manager_page_t;

typedef struct flash_manager flash_manager_t;

typedef enum
{
    FM_RESULT_SUCCESS, 
    FM_RESULT_ERROR_AREA_FULL, 
    FM_RESULT_ERROR_NOT_FOUND, 
    FM_RESULT_ERROR_FLASH_MALFUNCTION 
} fm_result_t;

typedef void (*flash_manager_write_complete_cb_t)(const flash_manager_t * p_manager,
        const fm_entry_t * p_entry,
        fm_result_t result);

typedef void (*flash_manager_invalidate_complete_cb_t)(const flash_manager_t * p_manager,
        fm_handle_t handle,
        fm_result_t result);

typedef void (*flash_manager_remove_complete_cb_t)(const flash_manager_t * p_manager);
typedef void (*flash_manager_queue_empty_cb_t)(void);
typedef struct
{
    const flash_manager_page_t *           p_area;                 
    uint32_t                               page_count;             
    uint32_t                               min_available_space;    
                                                                   
    flash_manager_write_complete_cb_t      write_complete_cb;      
    flash_manager_invalidate_complete_cb_t invalidate_complete_cb; 
    flash_manager_remove_complete_cb_t     remove_complete_cb;     
} flash_manager_config_t;
typedef struct
{
    fm_state_t state;          
    uint32_t invalid_bytes;    
    const fm_entry_t * p_seal; 
} flash_manager_internal_state_t;

struct flash_manager
{
    flash_manager_internal_state_t internal; 
    flash_manager_config_t config;        
};
typedef void (*flash_manager_mem_listener_cb_t)(void * p_args);
typedef struct
{
    queue_elem_t queue_elem; 
    flash_manager_mem_listener_cb_t callback; 
    void * p_args; 
} fm_mem_listener_t;
typedef enum
{
    FM_ITERATE_ACTION_STOP, 
    FM_ITERATE_ACTION_CONTINUE, 
} fm_iterate_action_t;
typedef fm_iterate_action_t (*flash_manager_read_cb_t)(const fm_entry_t * p_entry, void * p_args);
void flash_manager_init(void);
uint32_t flash_manager_add(flash_manager_t * p_manager,
        const flash_manager_config_t * p_config);
uint32_t flash_manager_remove(flash_manager_t * p_manager);
const fm_entry_t * flash_manager_entry_get(const flash_manager_t * p_manager, fm_handle_t handle);
uint32_t flash_manager_entry_read(const flash_manager_t * p_manager,
                                  fm_handle_t handle,
                                  void * p_data,
                                  uint32_t * p_length);
const fm_entry_t * flash_manager_entry_next_get(const flash_manager_t * p_manager,
        const fm_handle_filter_t * p_filter,
        const fm_entry_t * p_start);
uint32_t flash_manager_entries_read(const flash_manager_t * p_manager,
                                    const fm_handle_filter_t * p_filter,
                                    flash_manager_read_cb_t read_cb,
                                    void * p_args);
uint32_t flash_manager_entry_count_get(const flash_manager_t * p_manager, const fm_handle_filter_t * p_filter);
fm_entry_t * flash_manager_entry_alloc(flash_manager_t * p_manager,
        fm_handle_t handle,
        uint32_t data_length);
void flash_manager_entry_commit(const fm_entry_t * p_entry);
uint32_t flash_manager_entry_invalidate(flash_manager_t * p_manager, fm_handle_t handle);
void flash_manager_entry_release(fm_entry_t * p_entry);
void flash_manager_mem_listener_register(fm_mem_listener_t * p_listener);
bool flash_manager_is_stable(void);
void flash_manager_action_queue_empty_cb_set(flash_manager_queue_empty_cb_t queue_empty_cb);
const void * flash_manager_recovery_page_get(void);
static inline bool metadata_is_valid(const flash_manager_metadata_t * p_metadata)
{
    return (p_metadata->metadata_len != 0xFF &&
            p_metadata->metadata_len >= 8 &&
            p_metadata->page_index != 0xFF &&
            p_metadata->pages_in_area != 0xFF);
}
bool flash_manager_check_nvds_content(void);

#endif /* FLASH_MANAGER_H__ */
