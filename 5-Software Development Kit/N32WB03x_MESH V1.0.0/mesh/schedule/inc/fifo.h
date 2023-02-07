#ifndef MESH_FIFO_H__
#define MESH_FIFO_H__

#include <stdint.h>
#include <stdbool.h>



typedef void (*fifo_memcpy_t)(void* dest, const void* src);

typedef struct
{
    void*           elem_array;
    uint32_t        elem_size;          /**< Size of a single element in bytes. */
    uint32_t        array_len;          /**< Number of elements in the elem_array. Must be in the power of two. */
    uint32_t        head;               /**< Head index. Only for internal housekeeping. */
    uint32_t        tail;               /**< Tail index. Only for internal housekeeping. */
} fifo_t;

void fifo_init(fifo_t* p_fifo);
uint32_t fifo_push(fifo_t* p_fifo, const void* p_elem);
uint32_t fifo_pop(fifo_t* p_fifo, void* p_elem);
uint32_t fifo_peek_at(const fifo_t* p_fifo, void* p_elem, uint32_t elem);
uint32_t fifo_peek(const fifo_t* p_fifo, void* p_elem);
void fifo_flush(fifo_t* p_fifo);
uint32_t fifo_get_len(const fifo_t* p_fifo);
bool fifo_is_full(const fifo_t* p_fifo);
bool fifo_is_empty(const fifo_t* p_fifo);


#endif /* FIFO_H__ */
