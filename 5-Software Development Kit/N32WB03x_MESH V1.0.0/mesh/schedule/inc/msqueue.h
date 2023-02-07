#ifndef MSQUEUE_H__
#define MSQUEUE_H__

#include <stdint.h>
#include <stdbool.h>


typedef struct
{
   uint8_t stage_count; 
   uint8_t elem_size;   
   uint8_t elem_count;  
   uint8_t * p_stages;  
   void * p_elem_array; 
} msq_t;

void msq_init(msq_t * p_queue);
void * msq_get(const msq_t * p_queue, uint8_t stage);
void msq_move(msq_t * p_queue, uint8_t stage);
void msq_reset(msq_t * p_queue);
uint8_t msq_available(const msq_t * p_queue, uint8_t stage);

#endif /* MSQUEUE_H__ */
