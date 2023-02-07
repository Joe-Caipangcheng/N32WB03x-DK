
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
 * @file global_func.h
 * @author Nations Firmware Team
 * @version v1.0.3
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "n32wb03x.h"
#include "global_var.h"
#include "Typedefine.h"

#include "rwip_config.h"     // SW configuration
#include "compiler.h"
#include "co_version.h"      // version information
#include "co_utils.h"
#include "co_bt.h"           // Common BT Definitions
#include "co_math.h"         // Common Maths Definition
#include "co_bt_defines.h"


#include "arch.h"            // Platform architecture definition
#include "em_map.h"
#include "rf.h"

#include "rwip.h"            // RW definitions
#include "rwip_int.h"        // RW internal definitions
#include "rwip_task.h"       // Task definitions

#include "sch_alarm.h"       // for the half slot target ISR
#include "sch_arb.h"         // for the half us target ISR
#include "sch_prog.h"        // for the fifo/clock ISRs
#include "reg_ipcore.h"
#include "reg_blecore.h"
#include "aes.h"             // AES result function
#include "rwble.h"           // for sleep and wake-up specific functions
#include "lld.h"             // for AES encryption handler

#include "ke.h"              // kernel definition
#include "ke_event.h"        // kernel event
#include "ke_timer.h"        // definitions for timer
#include "ke_mem.h"          // kernel memory manager
#include "ke_task.h"         // Kernel Task


#include "rwble_hl.h"        // BLE HL definitions
#include "l2cc.h"
#include "llm_int.h"
#include "ahi_task.h"
#include "co_hci.h"
#include "hci.h"
#include "attm.h"
#include "atts.h"
#include "prf.h"
#include "gap.h"
#include "gapm.h"
#include "gapm_int.h"
#include "gapm_task.h"      // GAP Manager Task API
#include "gapc.h"
#include "gapc_task.h"      // GAP Controller Task API Definition
#include "gattm_int.h"
#include "gattc.h" 
#include "gatt.h"

#include "ble_stack_common.h"
#include "ns_adv_data_def.h"

#ifndef _GLOBAL_FUNC_H_
#define _GLOBAL_FUNC_H_

/* Public typedef -----------------------------------------------------------*/
typedef void (*IRQ_HANNDLE_FUN) (void);

/* Public define ------------------------------------------------------------*/ 
#define GLOBAL_INT_DISABLE()        \
uint32_t ui32IntStatus = 0;         \
do{                                 \
    ui32IntStatus = __get_PRIMASK();\
    __set_PRIMASK(1);               \
}while(0)

#define GLOBAL_INT_RESTORE()     \
do{                              \
    __set_PRIMASK(ui32IntStatus);\
}while(0)


/* Public constants ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/

extern void lpuart_init(void);
extern uint32_t calib_lsi_clk(void);
extern void EXTI_PA11_Configuration(void);
extern void pmu_hck_config(uint8_t select);



extern uint32_t ModuleIrqRegister(IRQn_Type irqn,IRQ_HANNDLE_FUN fun);
extern uint32_t ModuleIrqRemoval(IRQn_Type irqn);
extern void rwip_slp_isr(void);

extern void rf_tx_power_set(rf_tx_power_t pwr);
#endif    //_GLOBAL_FUNC_H_
