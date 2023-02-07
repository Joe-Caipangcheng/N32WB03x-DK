#ifndef MESH_FLASH_H__
#define MESH_FLASH_H__

#include <stdint.h>
#include <stdbool.h>


typedef enum
{
    FLASH_OP_TYPE_NONE  = 0x00, 
    FLASH_OP_TYPE_WRITE = 0x01, 
    FLASH_OP_TYPE_ERASE = 0x02, 
    FLASH_OP_TYPE_ALL   = 0x03  
} flash_op_type_t;


typedef struct
{
    uint32_t * p_start_addr; 
    uint32_t length;         
    const uint32_t * p_data; 
} flash_operation_params_write_t;
typedef struct
{
    uint32_t * p_start_addr; 
    uint32_t length; 
} flash_operation_params_erase_t;


typedef struct
{
    flash_op_type_t type;    
    union
    {
        flash_operation_params_write_t write; 
        flash_operation_params_erase_t erase; 
    } params;
} flash_operation_t;

typedef enum
{
    MESH_FLASH_USER_MESH, 
    MESH_FLASH_USER_APP,  
    MESH_FLASH_USERS      
} mesh_flash_user_t;







typedef void (*mesh_flash_op_cb_t)(mesh_flash_user_t user, const flash_operation_t * p_op, uint16_t token);










void mesh_flash_init(void);
uint32_t mesh_flash_op_push(mesh_flash_user_t user, const flash_operation_t * p_op, uint16_t * p_token);
void mesh_flash_user_callback_set(mesh_flash_user_t user, mesh_flash_op_cb_t cb);
void mesh_flash_set_suspended(bool suspend);


#endif /* MESH_FLASH_H__ */
