#ifndef FILTER_ENGINE_H__
#define FILTER_ENGINE_H__

#include <stdbool.h>
#include <stdint.h>

#include "mesh_list.h"
#include "scanner.h"


typedef enum
{
    FILTER_TYPE_PRE_PROC,   
    FILTER_TYPE_POST_PROC,  
    FILTER_TYPE_END,        
} filter_type_t;


typedef bool (* filter_handler_t)(scanner_packet_t * p_packet, void * p_data);

typedef struct
{
    filter_type_t    type;       
    filter_handler_t handler;    
    void *           p_data;     
    list_node_t      node;       
} filter_t;

void fen_filter_start(filter_t * p_filter);
void fen_filter_stop(filter_t * p_filter);
bool fen_filters_apply(filter_type_t type, scanner_packet_t * p_packet);
uint32_t fen_accepted_amount_get(filter_type_t type);


#endif /* FILTER_ENGINE_H__ */
