#ifndef __ACCESS_CONFIG_H__
#define __ACCESS_CONFIG_H__




#include "dsm_common.h"
#include "mesh_opt_access.h"
#include "access_internal.h"


typedef struct
{
    uint8_t is_metadata_stored : 1;
    uint8_t is_load_failed : 1;
    uint8_t is_restoring_ended : 1;
} local_access_status_t;

extern local_access_status_t access_status;
extern access_common_t access_model_pool[ACCESS_MODEL_COUNT];
extern access_element_t access_element_pool[ACCESS_ELEMENT_COUNT];
extern access_subscription_list_t access_subscription_list_pool[ACCESS_SUBSCRIPTION_LIST_COUNT];
extern uint8_t access_default_ttl;




uint32_t access_model_application_bind(access_model_handle_t handle, dsm_handle_t appkey_handle);
void model_store(access_model_handle_t handle);
uint32_t access_load_config_apply(void);


uint32_t access_model_id_get(access_model_handle_t handle, access_model_id_t * p_model_id);
uint32_t access_element_sig_model_count_get(uint16_t element_index, uint8_t * p_sig_model_count);
uint32_t access_element_models_get(uint16_t element_index, access_model_handle_t * p_models, uint16_t * p_count);
uint32_t access_element_location_get(uint16_t element_index, uint16_t * p_location);
uint32_t access_element_vendor_model_count_get(uint16_t element_index, uint8_t * p_vendor_model_count);
uint32_t access_handle_get(uint16_t element_index,access_model_id_t model_id,access_model_handle_t * p_handle);
uint32_t access_model_application_unbind(access_model_handle_t handle, dsm_handle_t appkey_handle);
uint32_t access_model_publish_application_get(access_model_handle_t handle, dsm_handle_t * p_appkey_handle);
uint32_t access_model_publication_stop(access_model_handle_t handle);
uint32_t access_model_publish_address_get(access_model_handle_t handle, dsm_handle_t * p_address_handle);
uint32_t access_model_publish_period_get(access_model_handle_t handle,
                                         access_publish_resolution_t * p_resolution,
                                         uint8_t * p_step_number);
uint32_t access_model_publish_address_set(access_model_handle_t handle, dsm_handle_t address_handle);                            
uint32_t access_model_publish_application_set(access_model_handle_t handle, dsm_handle_t appkey_handle);
uint32_t access_model_publish_friendship_credential_flag_set(access_model_handle_t handle, bool flag);
uint32_t access_model_publish_ttl_set(access_model_handle_t handle, uint8_t ttl);
uint32_t access_model_publish_friendship_credential_flag_get(access_model_handle_t handle, bool * p_flag);
uint32_t access_model_publish_ttl_get(access_model_handle_t handle, uint8_t * p_ttl);
uint32_t access_model_publish_retransmit_set(access_model_handle_t handle,
                                             access_publish_retransmit_t retransmit_params);
uint32_t access_model_publish_retransmit_get(access_model_handle_t handle,
                                             access_publish_retransmit_t * p_retransmit_params);                                             
uint32_t access_model_publish_period_set(access_model_handle_t handle,
                                         access_publish_resolution_t resolution,
                                         uint8_t step_number);                                             
uint32_t access_model_subscription_add(access_model_handle_t handle, dsm_handle_t address_handle);
uint32_t access_model_subscription_list_alloc(access_model_handle_t handle);
uint32_t access_model_subscription_remove(access_model_handle_t handle, dsm_handle_t address_handle);
uint32_t access_model_applications_get(access_model_handle_t handle, dsm_handle_t * p_appkey_handles, uint16_t * p_count);
uint32_t access_model_subscriptions_get(access_model_handle_t handle, dsm_handle_t * p_address_handles, uint16_t * p_count);


static inline bool model_handle_valid_and_allocated(access_model_handle_t handle)
{
    return (handle < ACCESS_MODEL_COUNT && ACCESS_INTERNAL_STATE_IS_ALLOCATED(access_model_pool[handle].internal_state));
}

static inline uint8_t access_default_ttl_get(void)
{
    return access_default_ttl;
}
static inline bool is_valid_opcode(access_opcode_t opcode)
{
    if (opcode.company_id == ACCESS_COMPANY_ID_NONE)
    {
        if ((opcode.opcode & 0xFF00) > 0)
        {
            return ((opcode.opcode & 0xC000) == 0x8000);
        }
        else
        {
            return ((opcode.opcode & 0xFF80) == 0x00) && (opcode.opcode != 0x7F);
        }
    }
    else
    {
        return ((opcode.opcode & 0xFFC0) == 0x00C0);
    }
}
static inline bool check_tx_params(access_model_handle_t handle, const access_message_tx_t * p_tx_message, const access_message_rx_t * p_rx_message, uint32_t * p_status)
{
    if (p_tx_message->length >= ACCESS_MESSAGE_LENGTH_MAX)
    {
        *p_status = MESH_ERROR_INVALID_LENGTH;
    }
    else if (!model_handle_valid_and_allocated(handle) ||
             access_model_pool[handle].model_info.element_index >= ACCESS_ELEMENT_COUNT)
    {
        *p_status = MESH_ERROR_NOT_FOUND;
    }
    else if ((p_rx_message == NULL &&
             (access_model_pool[handle].model_info.publish_appkey_handle  == DSM_HANDLE_INVALID ||
              access_model_pool[handle].model_info.publish_address_handle == DSM_HANDLE_INVALID)) ||
              !is_valid_opcode(p_tx_message->opcode))
    {
        *p_status = MESH_ERROR_INVALID_PARAM;
    }
    else
    {
        *p_status = MESH_SUCCESS;
    }

    return (MESH_SUCCESS == *p_status);
}

static inline uint32_t access_default_ttl_set(uint8_t ttl)
{
    if ((ttl > MESH_TTL_MAX) || (ttl == 1))
    {
        return MESH_ERROR_INVALID_PARAM;
    }
    else
    {
        access_default_ttl = ttl;
        return MESH_SUCCESS;
    }
}


#endif //__ACCESS_CONFIG_H__
