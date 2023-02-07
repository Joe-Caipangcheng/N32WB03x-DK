#ifndef __LIB_MESH_STACK_H__
#define __LIB_MESH_STACK_H__



#include <stdint.h>
#include "flash_manager.h"


/**
 * @brief  Initialize the mesh stack.
 * @param  model_init function used to initialize the application models.
 */
void mesh_stack_init(void (*model_init)(void));

/**
 * @brief  Mesh stack enables.
 * @param  scan_delay default delay is 1 second.
 */
void mesh_stack_enable(uint32_t scan_delay);






#endif //__LIB_MESH_STACK_H__




