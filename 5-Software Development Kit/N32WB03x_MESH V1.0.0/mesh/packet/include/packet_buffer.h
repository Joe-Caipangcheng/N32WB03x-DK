#ifndef PACKET_BUFFER_H__
#define PACKET_BUFFER_H__

#include <stdint.h>
#include <stdbool.h>

#include "packet.h"
#include "utils.h"

typedef enum
{
    PACKET_BUFFER_MEM_STATE_FREE,      
    PACKET_BUFFER_MEM_STATE_RESERVED,  
    PACKET_BUFFER_MEM_STATE_COMMITTED, 
    PACKET_BUFFER_MEM_STATE_POPPED,    
    PACKET_BUFFER_MEM_STATE_PADDING    
} packet_buffer_mem_state_t;


typedef struct
{
    uint16_t size;                       
    packet_buffer_mem_state_t packet_state; 
    uint8_t packet[] __attribute((aligned(WORD_SIZE)));
} packet_buffer_packet_t;

typedef struct
{
    uint16_t size;      
    uint16_t head;      
    uint16_t tail;      
    uint8_t * buffer;   
} packet_buffer_t;

uint16_t packet_buffer_max_packet_len_get(const packet_buffer_t * const p_buffer);
void packet_buffer_init(packet_buffer_t * p_buffer, void * const p_pool, const uint16_t pool_size);
void packet_buffer_flush(packet_buffer_t * p_buffer);
uint32_t packet_buffer_reserve(packet_buffer_t * const p_buffer, packet_buffer_packet_t ** pp_packet, uint16_t length);
void packet_buffer_commit(packet_buffer_t * const p_buffer, packet_buffer_packet_t * const p_packet, uint16_t length);
uint32_t packet_buffer_pop(packet_buffer_t * const p_buffer, packet_buffer_packet_t ** pp_packet);
bool packet_buffer_can_pop(packet_buffer_t * p_buffer);
bool packet_buffer_packets_ready_to_pop(packet_buffer_t * p_buffer);
void packet_buffer_free(packet_buffer_t * const p_buffer, packet_buffer_packet_t * const p_packet);
bool packet_buffer_is_empty(const packet_buffer_t * p_buffer);


#endif

