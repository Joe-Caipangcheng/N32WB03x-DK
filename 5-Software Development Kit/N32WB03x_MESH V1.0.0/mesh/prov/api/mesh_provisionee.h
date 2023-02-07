#ifndef MESH_PROVISIONEE_H__
#define MESH_PROVISIONEE_H__

#include <stdint.h>
#include <stdbool.h>

typedef void (*mesh_provisionee_prov_complete_cb_t)(void);
typedef void (*mesh_provisionee_prov_device_identification_start)(uint8_t attention_duration_s);
typedef void (*mesh_provisionee_prov_device_identification_stop)(void);
typedef void (*mesh_provisionee_prov_abort)(void);
typedef struct
{
    mesh_provisionee_prov_complete_cb_t prov_complete_cb;
    mesh_provisionee_prov_device_identification_start prov_device_identification_start_cb;
    mesh_provisionee_prov_device_identification_stop prov_device_identification_stop_cb;
    mesh_provisionee_prov_abort prov_abort_cb;
    const char * p_device_uri;
    const uint8_t * p_static_data;
} mesh_provisionee_start_params_t;

uint32_t mesh_provisionee_prov_start(const mesh_provisionee_start_params_t * p_start_params);
uint32_t mesh_provisionee_prov_listen_stop(void);



#endif /* MESH_PROVISIONEE_H__ */
