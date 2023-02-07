/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file mesh_app.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "mesh_app.h"
#include "mesh_app_task.h"
#include "n32wb03x.h"
#include "gapm_task.h"
#include "rwip.h"
#include "co_bt_defines.h"
#include "gapc_task.h" 
#include "ke_task.h" 
#include "mesh_app.h"
#include "mesh_app_task.h"
#include "rwip.h"
#include "app_task.h"
#include "rwip_int.h"
#include "mesh.h"
#include "flash_manager.h"

/**
 * @brief  Reset the flash sectors.
 */
void ns_mesh_nvds_clear(void)
{
    Qflash_Erase_Sector(0x103B000);
    Qflash_Erase_Sector(0x103C000);
    Qflash_Erase_Sector(0x103D000);
    Qflash_Erase_Sector(0x103E000);
    Qflash_Erase_Sector(0x103F000);  
}
/**
 * @brief  Initialize the mesh stack.
 * @param  model_init function used to initialize the application models.
 */
void ns_mesh_stack_init(void (*model_init)(void))
{
    if(flash_manager_check_nvds_content() != 0){
        
        ns_mesh_nvds_clear();
        NVIC_SystemReset();        
        while(1);
    }    
    ke_task_delete(TASK_MESH);
    ke_task_create(TASK_MESH, &TASK_DESC_MESH_M);    
    mesh_stack_init(model_init);
}

/**
 * @brief  Mesh stack enables.
 */
void ns_mesh_stack_enable(void)
{
    mesh_stack_enable(1000);

}





