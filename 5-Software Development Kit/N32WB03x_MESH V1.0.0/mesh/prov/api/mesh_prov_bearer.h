#ifndef MESH_PROV_BEARER_H__
#define MESH_PROV_BEARER_H__

#include <stdint.h>
#include "mesh_list.h"

typedef enum
{
    MESH_PROV_LINK_CLOSE_REASON_SUCCESS = 0, 
    MESH_PROV_LINK_CLOSE_REASON_TIMEOUT = 1, 
    MESH_PROV_LINK_CLOSE_REASON_ERROR   = 2, 
    MESH_PROV_LINK_CLOSE_REASON_LAST    = MESH_PROV_LINK_CLOSE_REASON_ERROR 
} mesh_prov_link_close_reason_t;

typedef struct prov_bearer prov_bearer_t;

typedef uint32_t (*prov_bearer_if_tx_t)(prov_bearer_t * p_bearer, const uint8_t * p_data, uint16_t length);

typedef uint32_t (*prov_bearer_if_listen_start_t)(prov_bearer_t *       p_bearer,
                                                  const char * p_uri,
                                                  uint16_t     oob_info,
                                                  uint32_t     link_timeout_us);

typedef uint32_t (*prov_bearer_if_listen_stop_t)(prov_bearer_t * p_bearer);

typedef uint32_t (*prov_bearer_if_link_open_t)(prov_bearer_t * p_bearer,
                                               const uint8_t * p_uuid,
                                               uint32_t        link_timeout_us);

typedef void (*prov_bearer_if_link_close_t)(prov_bearer_t *                   p_bearer,
                                            mesh_prov_link_close_reason_t close_reason);

typedef struct
{
    prov_bearer_if_tx_t tx;
    prov_bearer_if_listen_start_t listen_start;
    prov_bearer_if_listen_stop_t listen_stop;
    prov_bearer_if_link_open_t link_open;
    prov_bearer_if_link_close_t link_close;
} prov_bearer_interface_t;

typedef void (*prov_bearer_cb_rx_t)(prov_bearer_t * p_bearer,
                                    const uint8_t * p_data,
                                    uint16_t        length);

typedef void (*prov_bearer_cb_ack_t)(prov_bearer_t * p_bearer);

typedef void (*prov_bearer_cb_link_opened_t)(prov_bearer_t * p_bearer);

typedef void (*prov_bearer_cb_link_closed_t)(prov_bearer_t *                   p_bearer,
                                             mesh_prov_link_close_reason_t reason);

typedef struct
{
    prov_bearer_cb_rx_t rx;
    prov_bearer_cb_ack_t ack;
    prov_bearer_cb_link_opened_t opened;
    prov_bearer_cb_link_closed_t closed;
} prov_bearer_callbacks_t;
#define MESH_PROV_BEARER_COUNT (3)

typedef enum
{
    MESH_PROV_BEARER_ADV  = 0x01, 
    MESH_PROV_BEARER_GATT = 0x02, 
    MESH_PROV_BEARER_MESH = 0x04  
} mesh_prov_bearer_type_t;

struct prov_bearer
{
    list_node_t node;                            
    mesh_prov_bearer_type_t     bearer_type;
    const prov_bearer_interface_t * p_interface; 
    const prov_bearer_callbacks_t * p_callbacks; 
    void * p_parent;                             
};

#endif
