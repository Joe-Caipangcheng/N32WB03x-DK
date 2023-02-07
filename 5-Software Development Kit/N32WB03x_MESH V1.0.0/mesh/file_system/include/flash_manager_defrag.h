#ifndef FLASH_MANAGER_DEFRAG_H__
#define FLASH_MANAGER_DEFRAG_H__

#include "flash_manager.h"

bool flash_manager_defrag_init(void);
bool flash_manager_defragging(const flash_manager_t * p_manager);
bool flash_manager_defrag_is_running(void);
const flash_manager_page_t * flash_manager_defrag_page_get(void);
void flash_manager_defrag(const flash_manager_t * p_manager);
const void * flash_manager_defrag_recovery_page_get(void);


#endif /* FLASH_MANAGER_DEFRAG_H__ */

