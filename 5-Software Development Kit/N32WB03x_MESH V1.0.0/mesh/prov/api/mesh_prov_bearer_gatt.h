#ifndef MESH_PROV_BEARER_GATT_H__
#define MESH_PROV_BEARER_GATT_H__

#include <stdint.h>
#include "fsm.h"
#include "mesh_prov_bearer.h"
#include "bearer_event.h"




typedef struct
{
    fsm_t fsm;
    uint16_t conn_index;
    timer_event_t link_timeout_event;
    uint32_t link_timeout_us;
    bearer_event_sequential_t bearer_event_seq;
    prov_bearer_t bearer;
} mesh_prov_bearer_gatt_t;

uint32_t mesh_prov_bearer_gatt_init(mesh_prov_bearer_gatt_t * p_bearer_gatt);
prov_bearer_t * mesh_prov_bearer_gatt_interface_get(mesh_prov_bearer_gatt_t * p_bearer_gatt);


#endif /* MESH_PROV_BEARER_GATT_H__ */
