#ifndef CORE_TX_ADV_CONFIG_H__
#define CORE_TX_ADV_CONFIG_H__



#include "dsm_common.h"
#include "advertiser.h"

typedef struct
{
    uint32_t adv_tx_count;
    advertiser_t advertiser;
} adv_bearer_role_t;
extern adv_bearer_role_t m_bearer_roles[CORE_TX_ROLE_COUNT];


static inline uint32_t mesh_opt_core_adv_set(core_tx_role_t index, const mesh_opt_core_adv_t * p_entry)  
{                                                                 
    if (index >= (CORE_TX_ROLE_COUNT)) return MESH_ERROR_INVALID_PARAM;     
    mesh_config_entry_id_t id = MESH_OPT_CORE_ADV_EID;                               
    id.record += (uint16_t) index;                                
    return mesh_config_entry_set(id, p_entry);                    
}                                                                 
static inline uint32_t mesh_opt_core_adv_get(core_tx_role_t index, mesh_opt_core_adv_t * p_entry)        
{                                                                 
    if (index >= (CORE_TX_ROLE_COUNT)) return MESH_ERROR_INVALID_PARAM;     
    mesh_config_entry_id_t id = MESH_OPT_CORE_ADV_EID;                               
    id.record += (uint16_t) index;                                
    return mesh_config_entry_get(id, p_entry);                    
}                                                                 
static inline uint32_t mesh_opt_core_adv_delete(core_tx_role_t index)                          
{                                                                 
    if (index >= (CORE_TX_ROLE_COUNT)) return MESH_ERROR_INVALID_PARAM;     
    mesh_config_entry_id_t id = MESH_OPT_CORE_ADV_EID;                               
    id.record += (uint16_t) index;                                
    return mesh_config_entry_delete(id);                          
}




#endif /* CORE_TX_ADV_CONFIG_H__ */
