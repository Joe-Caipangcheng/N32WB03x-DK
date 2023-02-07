#ifndef MESH_ADV_H__
#define MESH_ADV_H__

#include <stdint.h>


enum
{
    UNIT_0_625_MS = 625,        
    UNIT_1_25_MS  = 1250,       
    UNIT_10_MS    = 10000       
};

#define MSEC_TO_UNITS(TIME, RESOLUTION) (((TIME) * 1000) / (RESOLUTION))

#define MESH_SOFTDEVICE_CONN_CFG_TAG (1)

#define MESH_ADV_INTERVAL_DEFAULT (MSEC_TO_UNITS(200, UNIT_0_625_MS))
#define MESH_ADV_TIMEOUT_INFINITE  (0)


void prov_proxy_adv_data_set(uint16_t service_uuid, const uint8_t * p_service_data, uint8_t length);
void prov_proxy_adv_params_set(uint32_t timeout_ms, uint32_t interval_units);
void prov_proxy_adv_start(void);
void prov_proxy_adv_stop(void);
void prov_proxy_adv_delete(void);

#endif /* MESH_ADV_H__ */
