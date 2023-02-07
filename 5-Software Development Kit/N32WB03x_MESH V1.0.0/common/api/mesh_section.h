#ifndef MESH_SECTION_H__
#define MESH_SECTION_H__

#include "mesh_common.h"




#define SECTION_START_ADDR(section_name)       &CONCAT_2(section_name, $$Base)
#define SECTION_END_ADDR(section_name)         &CONCAT_2(section_name, $$Limit)
#define SECTION_LENGTH(section_name)                        \
    ((size_t)SECTION_END_ADDR(section_name) -               \
     (size_t)SECTION_START_ADDR(section_name))
#define SECTION_DEF(section_name, data_type)                \
    extern data_type * CONCAT_2(section_name, $$Base);          \
    extern void      * CONCAT_2(section_name, $$Limit)
#define SECTION_ITEM_REGISTER(section_name, section_var) \
    section_var __attribute__ ((section(STRINGIFY(section_name)))) __attribute__((used))
#define SECTION_ITEM_GET(section_name, data_type, i) \
    ((data_type*)SECTION_START_ADDR(section_name) + (i))
#define SECTION_ITEM_COUNT(section_name, data_type) \
    SECTION_LENGTH(section_name) / sizeof(data_type)

    
    
    
#define MESH_SECTION_START(section_name) SECTION_START_ADDR(section_name)
#define MESH_SECTION_END(section_name)   SECTION_END_ADDR(section_name)

#define MESH_SECTION_ITEM_REGISTER_FLASH(section_name, section_var) SECTION_ITEM_REGISTER(section_name, section_var)
#define MESH_SECTION_ITEM_REGISTER_RAM(section_name, section_var) SECTION_ITEM_REGISTER(section_name, section_var)

#define MESH_SECTION_DEF_FLASH(section_name, data_type) SECTION_DEF(section_name, data_type)
#define MESH_SECTION_DEF_RAM(section_name, data_type) SECTION_DEF(section_name, data_type)

#define MESH_SECTION_LENGTH(section_name) SECTION_LENGTH(section_name)

#define MESH_SECTION_ITEM_GET(section_name, data_type, i) SECTION_ITEM_GET(section_name, data_type, i)
#define MESH_SECTION_ITEM_COUNT(section_name, data_type) SECTION_ITEM_COUNT(section_name, data_type)
        
    

#define MESH_SECTION_FOR_EACH(section_name, data_type, variable)                               \
    for (data_type * variable = (data_type *) MESH_SECTION_START(section_name);                \
         (intptr_t) variable != (intptr_t) MESH_SECTION_END(section_name);                         \
         variable++)    
    
    
    
    
    
    
    
#endif // MESH_SECTION_H__
