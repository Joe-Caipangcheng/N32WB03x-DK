#ifndef AD_LISTENER_H_
#define AD_LISTENER_H_

#include <stdint.h>
#include "packet.h"
#include "mesh.h"
#include "scanner.h"
#include "mesh_section.h"


#define ADL_WILDCARD_AD_TYPE   0u
#define ADL_WILDCARD_ADV_TYPE  (ble_packet_type_t)0xFFu

typedef void (* ad_handler_t)(const uint8_t * p_packet,
                              uint32_t ad_packet_length,
                              const mesh_rx_metadata_t * p_metadata);

typedef struct
{
    uint8_t           ad_type;
    ble_packet_type_t adv_packet_type;
    ad_handler_t      handler;
} ad_listener_t;

#define AD_LISTENER(name) MESH_SECTION_ITEM_REGISTER_FLASH(ad_listeners, const ad_listener_t name)

void ad_listener_init(void);
void ad_listener_process(ble_packet_type_t adv_type, const uint8_t * p_payload, uint32_t payload_length, const mesh_rx_metadata_t * p_metadata);


#endif /* AD_LISTENER_H_ */
