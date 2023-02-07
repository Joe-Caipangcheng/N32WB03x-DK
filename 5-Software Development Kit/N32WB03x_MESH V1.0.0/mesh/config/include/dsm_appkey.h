#ifndef __DSM_APPKEY_H__
#define __DSM_APPKEY_H__




#include "dsm_common.h"
#include "dsm_devkey.h"


extern appkey_t dsm_appkeys[DSM_APP_MAX];
extern uint32_t dsm_appkey_allocated[BITFIELD_BLOCK_COUNT(DSM_APP_MAX)];









void appkey_set(mesh_key_index_t app_key_index, dsm_handle_t subnet_handle, const uint8_t * p_key, dsm_handle_t handle);
bool app_key_handle_get(mesh_key_index_t app_key_index, dsm_handle_t * p_handle);
void get_app_secmat(dsm_handle_t subnet_handle, uint8_t aid, const mesh_application_secmat_t ** pp_app_secmat);
dsm_handle_t dsm_appkey_index_to_appkey_handle(mesh_key_index_t appkey_index);
uint32_t dsm_appkey_handle_to_appkey_index(dsm_handle_t appkey_handle, mesh_key_index_t * p_index);
uint32_t dsm_appkey_add(mesh_key_index_t app_key_index, dsm_handle_t subnet_handle, const uint8_t * p_key, dsm_handle_t * p_app_handle);
uint32_t dsm_appkey_delete(dsm_handle_t app_handle);



static inline dsm_handle_t get_app_handle(const mesh_application_secmat_t * p_secmat)
{
    if (p_secmat >= &dsm_appkeys[0].secmat &&
        p_secmat <= &dsm_appkeys[DSM_APP_MAX - 1].secmat)
    {
        return (((uint32_t) p_secmat - (uint32_t) &dsm_appkeys[0].secmat) / sizeof(appkey_t));
    }
    else if (p_secmat >= &dsm_devkeys[0].secmat &&
             p_secmat <= &dsm_devkeys[DSM_DEVICE_MAX - 1].secmat)
    {
        return DSM_DEVKEY_HANDLE_START + (((uint32_t) p_secmat - (uint32_t) &dsm_devkeys[0].secmat) / sizeof(devkey_t));

    }
    return DSM_HANDLE_INVALID;
}
static inline dsm_handle_t dsm_appkey_handle_get(const mesh_application_secmat_t * p_secmat)
{
    if (p_secmat == NULL)
    {
        return DSM_HANDLE_INVALID;
    }
    return get_app_handle(p_secmat);
}


#endif //__DSM_APPKEY_H__
