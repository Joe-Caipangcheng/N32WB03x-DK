#ifndef __SCANNER_H__
#define __SCANNER_H__







#include "mesh.h"
#include "packet.h"
#include "bearer_event.h"




#define SCANNER_BUFFER_SIZE 256
#define SCANNER_CHANNELS_MAX           (3)
#define SCANNER_ACCESS_ADDR_INVALID    (0x00000000)
#define SCANNER_CHANNELS_DEFAULT       {37, 38, 39}


typedef struct
{
    mesh_rx_metadata_scanner_t metadata; 
    packet_t packet;                         
} scanner_packet_t;

typedef struct
{
    uint32_t successful_receives;          
    uint32_t crc_failures;                 
    uint32_t length_out_of_bounds;         
    uint32_t out_of_memory;                
} scanner_stats_t;
typedef void (*scanner_rx_callback_t)(const scanner_packet_t * p_packet);


//void scanner_init(bearer_event_flag_callback_t packet_process_cb);
void scanner_init(void (*packet_process_cb)(void));
void scanner_handle_end_event(mesh_rx_metadata_scanner_t *metadata, uint8_t *adv_pdu, uint16_t len);
void scanner_radio_start(void);
void scanner_radio_stop(void);
const scanner_packet_t * scanner_rx(void);
void scanner_packet_release(const scanner_packet_t * p_packet);
bool scanner_rx_pending(void);
bool scanner_radio_is_idle(void);
void scanner_radio_delete(void);
void scanner_radio_create(void);


#endif //__SCANNER_H__
