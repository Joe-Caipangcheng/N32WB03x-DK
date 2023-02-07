#ifndef MESH_GATT_H__
#define MESH_GATT_H__

#include <stdint.h>
#include "packet_buffer.h"
#include "mesh_gatt_srv.h"
#include "mesh.h"


#define MESH_GATT_RX_SAR_TIMEOUT_US 20000000
#define MESH_GATT_CONNECTION_COUNT_MAX (1)

#define MESH_GATT_PROXY_PDU_MAX_SIZE (66)
#define MESH_GATT_MTU_SIZE_MAX       (69)
#define MESH_GATT_PACKET_MAX_SIZE    (MESH_GATT_PROXY_PDU_MAX_SIZE - 1)
#define MESH_GATT_TX_BUFFER_SIZE     ALIGN_VAL(MESH_GATT_PACKET_MAX_SIZE + \
                                               sizeof(packet_buffer_packet_t), WORD_SIZE)*3


#define MESH_PB_GATT_SERVICE_UUID  (0x1827)
#define MESH_PB_GATT_CHAR_IN_UUID  (0x2ADB)
#define MESH_PB_GATT_CHAR_OUT_UUID (0x2ADC)                                               
                                               
#define PROXY_UUID_SERVICE 0x1828
#define PROXY_UUID_CHAR_TX 0x2ade
#define PROXY_UUID_CHAR_RX 0x2add                                    

                                               
                                            
                                               
typedef enum
{
    MESH_GATT_PDU_TYPE_NETWORK_PDU,
    MESH_GATT_PDU_TYPE_MESH_BEACON,
    MESH_GATT_PDU_TYPE_PROXY_CONFIG,
    MESH_GATT_PDU_TYPE_PROV_PDU,
    MESH_GATT_PDU_TYPE_PROHIBITED,
    MESH_GATT_PDU_TYPE_INVALID = 0xFF
} mesh_gatt_pdu_type_t;

typedef enum
{
    MESH_GATT_EVT_TYPE_RX,
    MESH_GATT_EVT_TYPE_TX_COMPLETE,
    MESH_GATT_EVT_TYPE_ADV_TIMEOUT,
    MESH_GATT_EVT_TYPE_CONNECTED,
    MESH_GATT_EVT_TYPE_DISCONNECTED,
    MESH_GATT_EVT_TYPE_TX_READY,
} mesh_gatt_evt_type_t;

typedef struct
{
    mesh_gatt_pdu_type_t pdu_type;
    const uint8_t * p_data;
    uint16_t length;
} mesh_gatt_evt_rx_t;

typedef struct
{
    mesh_tx_token_t token;
    mesh_gatt_pdu_type_t pdu_type;
} mesh_gatt_evt_tx_complete_t;

typedef struct
{
    mesh_gatt_evt_type_t type;
    uint8_t conn_index;
    union
    {
        mesh_gatt_evt_rx_t rx;
        mesh_gatt_evt_tx_complete_t tx_complete;
    } params;
} mesh_gatt_evt_t;


typedef struct
{
    uint16_t service;
    uint16_t tx_char;
    uint16_t rx_char;
} mesh_gatt_uuids_t;


typedef struct
{
    packet_buffer_packet_t * p_curr_packet;
    uint8_t offset;
} mesh_gatt_transaction_t;


typedef struct
{
    uint16_t conn_handle;
    uint16_t effective_mtu;
    struct
    {
        packet_buffer_t packet_buffer;
        uint8_t packet_buffer_data[MESH_GATT_TX_BUFFER_SIZE];
        mesh_gatt_transaction_t transaction;
        bool tx_complete_process;
    } tx;
    struct
    {
        uint8_t buffer[MESH_GATT_PACKET_MAX_SIZE];
        mesh_gatt_pdu_type_t pdu_type;
        uint8_t offset;
        timer_event_t timeout_event;
    } rx;
} mesh_gatt_connection_t;

typedef void (*mesh_gatt_evt_handler_t)(const mesh_gatt_evt_t * p_evt, void * p_context);

struct mesh_gatt
{
    mesh_gatt_uuids_t uuids;
    mesh_gatt_evt_handler_t evt_handler;
    mesh_gatt_connection_t connections;
    void * p_context;
};


typedef struct mesh_gatt mesh_gatt_t;


extern const struct app_subtask_handlers mesh_gatt_app_handlers;

typedef uint32_t mesh_tx_token_t;

void mesh_gatt_init(const mesh_gatt_uuids_t * p_uuids, mesh_gatt_evt_handler_t evt_handler, void * p_context);
uint32_t mesh_gatt_disconnect(uint16_t conn_index);
uint8_t * mesh_gatt_packet_alloc(uint16_t conn_index,mesh_gatt_pdu_type_t type,uint16_t size,mesh_tx_token_t token);
uint32_t mesh_gatt_packet_send(uint16_t conn_index, const uint8_t * p_packet);
void mesh_gatt_packet_discard(uint16_t conn_index, const uint8_t * p_packet);
bool mesh_gatt_packet_is_pending(uint16_t conn_index);





#endif /* MESH_GATT_H__ */
