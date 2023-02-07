#ifndef BEARER_HANDLER_H__
#define BEARER_HANDLER_H__

#include <stdint.h>
#include <stdbool.h>
#include "mesh_queue.h"




typedef void (*bearer_start_cb_t)(void* p_args);


typedef struct
{
    bearer_start_cb_t          start_cb;          
    void*                      p_args;            
    queue_elem_t               queue_elem;        
} bearer_action_t;

typedef void (*bearer_handler_stopped_cb_t)(void);
void bearer_handler_init(void);
uint32_t bearer_handler_start(void);
uint32_t bearer_handler_stop(bearer_handler_stopped_cb_t cb);
uint32_t bearer_handler_action_enqueue(bearer_action_t* p_action);
void bearer_handler_wake_up(void);
uint32_t bearer_handler_action_fire(bearer_action_t* p_action);
void bearer_handler_action_end(void);



#endif /* BEARER_HANDLER_H__ */

