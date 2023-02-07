#ifndef NRF_MESH_SECTION_H__
#define NRF_MESH_SECTION_H__

#include "mesh_section.h"




#define NRF_MESH_SECTION_START(section_name) NRF_SECTION_START_ADDR(section_name)
#define NRF_MESH_SECTION_END(section_name)   NRF_SECTION_END_ADDR(section_name)

#define NRF_MESH_SECTION_ITEM_REGISTER_FLASH(section_name, section_var) NRF_SECTION_ITEM_REGISTER(section_name, section_var)
#define NRF_MESH_SECTION_ITEM_REGISTER_RAM(section_name, section_var) NRF_SECTION_ITEM_REGISTER(section_name, section_var)

#define NRF_MESH_SECTION_DEF_FLASH(section_name, data_type) NRF_SECTION_DEF(section_name, data_type)
#define NRF_MESH_SECTION_DEF_RAM(section_name, data_type) NRF_SECTION_DEF(section_name, data_type)

#define NRF_MESH_SECTION_LENGTH(section_name) NRF_SECTION_LENGTH(section_name)

#define NRF_MESH_SECTION_ITEM_GET(section_name, data_type, i) NRF_SECTION_ITEM_GET(section_name, data_type, i)
#define NRF_MESH_SECTION_ITEM_COUNT(section_name, data_type) NRF_SECTION_ITEM_COUNT(section_name, data_type)
        
    

#define NRF_MESH_SECTION_FOR_EACH(section_name, data_type, variable)                               \
    for (data_type * variable = (data_type *) NRF_MESH_SECTION_START(section_name);                \
         (intptr_t) variable != (intptr_t) NRF_MESH_SECTION_END(section_name);                         \
         variable++)


#endif /* NRF_MESH_SECTION_H__ */
