#ifndef MESH_QUEUE_H__
#define MESH_QUEUE_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "utils.h"


typedef struct queue_elem
{
    void* p_data;               /**< Pointer to data representing the queue element. Never changed by the queue module. */
    struct queue_elem* p_next;  /**< Pointer to the next element in the queue. Set and used by the queue module. */
} queue_elem_t;

typedef struct
{
    queue_elem_t* p_front; /**< Pointer to the front of the queue, where the queue elements are popped. */
    queue_elem_t* p_back;  /**< Pointer to the back of the queue, where the queue elements are pushed. */
} queue_t;

void queue_init(queue_t* p_queue);
void queue_push(queue_t* p_queue, queue_elem_t* p_elem);
queue_elem_t* queue_pop(queue_t* p_queue);
queue_elem_t* queue_peek(const queue_t* p_queue);
void queue_merge(queue_t * p_dst, queue_t * p_src);
typedef struct
{
    queue_t * p_queue; /**< Queue this iterator is iterating over. */
    queue_elem_t ** pp_elem; /**< Double pointer to the current iterator element. */
    bool repeat; /**< Whether to repeat the same element again. */
} queue_elem_iterator_t;
#define QUEUE_ITERATOR_BEGIN(p_QUEUE)   \
    {                                   \
        .p_queue = (p_QUEUE),           \
        .pp_elem = &(p_QUEUE)->p_front, \
        .repeat = false                 \
    }

#define QUEUE_ITERATOR_END(p_QUEUE)   \
    {                                   \
        .p_queue = (p_QUEUE),           \
        .pp_elem = &(p_QUEUE)->p_back, \
        .repeat = false                 \
    }

#define QUEUE_FOREACH(p_QUEUE, ITERATOR)                                 \
    for (queue_elem_iterator_t ITERATOR = QUEUE_ITERATOR_BEGIN(p_QUEUE); \
         (*((ITERATOR).pp_elem) != NULL);                                \
         queue_iterator_iterate(&ITERATOR))

void queue_iterator_elem_remove(queue_elem_iterator_t * p_it);
void queue_iterator_elem_insert(queue_elem_iterator_t * p_it, queue_elem_t * p_elem);
void queue_iterator_iterate(queue_elem_iterator_t * p_it);

#endif /* MESH_QUEUE_H__ */

