#ifndef MESH_PACKET_H__
#define MESH_PACKET_H__

#include <stdint.h>
#include <string.h>

#include "mesh_error.h"
#include "utils.h"

#define BLE_GAP_ADDR_LEN (6)

#define BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH    (31)                
#define BLE_ADV_PACKET_OVERHEAD              (BLE_GAP_ADDR_LEN)  
#define BLE_ADV_PACKET_HEADER_LENGTH         (2)                 
#define BLE_ADV_PACKET_HEADER_PADDING_LENGTH (1)                 
#define BLE_ADV_PACKET_HEADER_BUFFER_LENGTH                                                        \
    (BLE_ADV_PACKET_HEADER_LENGTH +                                                                \
     BLE_ADV_PACKET_HEADER_PADDING_LENGTH) 
#define BLE_ADV_PACKET_BUFFER_OVERHEAD                                                             \
    (BLE_ADV_PACKET_HEADER_BUFFER_LENGTH +                                                         \
     BLE_ADV_PACKET_OVERHEAD) 
#define BLE_ADV_PACKET_BUFFER_MAX_LENGTH                                                           \
    (BLE_ADV_PACKET_HEADER_BUFFER_LENGTH + BLE_ADV_PACKET_OVERHEAD +                               \
     BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH) 
#define BLE_ADV_PACKET_MIN_LENGTH                                                                  \
    (BLE_ADV_PACKET_OVERHEAD) 
#define BLE_AD_DATA_OVERHEAD                 (1)             
#define BLE_ADV_SERVICE_DATA_UUID_DFU        (0xfee4)        

#define AD_TYPE_MESH                        (0x2a)           
#define AD_TYPE_BEACON                      (0x2b)           
#define AD_TYPE_PB_ADV                      (0x29)           
#define AD_TYPE_DFU                         (0x16)           


typedef void packet_generic_t;


typedef enum
{
    BLE_PACKET_TYPE_ADV_IND,           
    BLE_PACKET_TYPE_ADV_DIRECT_IND,    
    BLE_PACKET_TYPE_ADV_NONCONN_IND,   
    BLE_PACKET_TYPE_SCAN_REQ,          
    BLE_PACKET_TYPE_SCAN_RSP,          
    BLE_PACKET_TYPE_CONN_REQ,          
    BLE_PACKET_TYPE_ADV_DISCOVER_IND,  
    BLE_PACKET_TYPE_ADV_EXT            
} ble_packet_type_t;

typedef struct __attribute((packed))
{
    uint8_t type        : 4;            
    uint8_t _rfu1       : 2;            
    uint8_t addr_type   : 1;            
    uint8_t _rfu2       : 1;            
    uint8_t length;                     
    uint8_t _rfu3;                      
} ble_packet_hdr_t;

typedef struct __attribute((packed))
{
    uint8_t length;                     
    uint8_t type;                       
} ble_ad_header_t;

typedef struct __attribute((packed))
{
    uint8_t length;                     
    uint8_t type;                       
    uint8_t data[];                     
} ble_ad_data_t;

typedef struct __attribute((packed))
{
    uint16_t uuid;      
    uint8_t data[];     
} ble_ad_data_service_data_t;


typedef struct __attribute((packed))
{
    ble_packet_hdr_t header;                            
    uint8_t addr[BLE_GAP_ADDR_LEN];                     
    uint8_t payload[BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH]; 
} packet_t;


static inline ble_ad_data_t* packet_ad_type_get_next(ble_ad_data_t* p_ad)
{
    return (ble_ad_data_t*) ((uint8_t*) p_ad + p_ad->length + BLE_AD_DATA_OVERHEAD);
}

static inline void packet_payload_size_set(packet_t * p_packet, uint8_t size)
{
    p_packet->header.length = size + BLE_GAP_ADDR_LEN;
}

static inline uint8_t packet_payload_size_get(const packet_t * p_packet)
{
    return p_packet->header.length - BLE_GAP_ADDR_LEN;
}



#endif

