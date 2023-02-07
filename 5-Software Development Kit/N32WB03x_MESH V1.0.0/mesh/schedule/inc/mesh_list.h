#ifndef MESH_LIST_H__
#define MESH_LIST_H__

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include <mesh_error.h>


#define LIST_FOREACH(ITEM, LIST)                                        \
    for (const list_node_t * (ITEM) = (LIST); (ITEM) != NULL; (ITEM) = (ITEM)->p_next)

typedef struct list_node
{
    struct list_node * p_next;
} list_node_t;

typedef bool (*list_cmp_cb_t)(const list_node_t * p1, const list_node_t * p2);

static inline void list_insert(list_node_t * p_node, list_node_t * p_new)
{
    if (p_node != p_new)
    {
        p_new->p_next  = p_node->p_next;
        p_node->p_next = p_new;
    }
}

void list_add(list_node_t ** pp_head, list_node_t * p_new);

uint32_t list_remove(list_node_t ** pp_head, list_node_t * p_node);

uint32_t list_size_get(list_node_t * p_head);

void list_sorted_add(list_node_t ** pp_head, list_node_t * p_new, list_cmp_cb_t cmp_cb);

uint32_t list_compare_add(list_node_t ** pp_head, list_node_t * p_new, list_cmp_cb_t cmp_cb);



#endif
