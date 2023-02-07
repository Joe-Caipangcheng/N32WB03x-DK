
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
 * @file ble_stack_common.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __BLE_STACK_COMMON_H_
#define __BLE_STACK_COMMON_H_


#include "rwip.h"
#include "global_var.h"
#include "global_func.h"
#include "Typedefine.h"
#include "toolchain.h"


#ifdef __cplusplus
extern "C" {
#endif
extern void rwip_fifo_isr(void);
/* Public define ------------------------------------------------------------*/
/* Public typedef -----------------------------------------------------------*/
/* Public define ------------------------------------------------------------*/

#if defined (MCU_LSC_SEL_LSE)
/* Select the LSE as Low speed clock source, don't change the LSC after this*/
#define NS_BLE_STACK_INIT()                                 \
do{                                                         \
        REG32(0x40007030)       = 0;                        \
		g_lsi_1_syscle_cnt_value = calib_lsi_clk();         \
		g_modem_flag            = MODEM_CHIP;               \
		g_app_init_branch       = false;                    \
		g_clock_src             = HSI_CLK;                  \
		g_system_hsi_clk        = HSI_VALUE;                \
		g_cal_hsi_cnt_value     = HSI_64M_VAL;              \
		g_delay_div_param       = 110;                      \
		g_delay_us_div_param    = 11;                       \
        llm_local_le_feats.feats[0] = 0xff;                 \
        llm_local_le_feats.feats[1] = 0xff;                 \
        llm_local_le_feats.feats[2] = 0xff;                 \
        llm_local_le_feats.feats[3] = 0x07;                 \
        llm_local_le_feats.feats[4] = 0x00;                 \
        llm_local_le_feats.feats[5] = 0x00;                 \
        llm_local_le_feats.feats[6] = 0x00;                 \
        llm_local_le_feats.feats[7] = 0x00;                 \
		REG32(0x200000dc) =(uint32_t) prf_add_profile;      \
		REG32(0x200000e0) =(uint32_t) prf_get_task_from_id; \
		REG32(0x200000e4) =(uint32_t) prf_get_id_from_task; \
        RCC->LSCTRL |= (1<<2);                              \
        while(!(RCC->LSCTRL & (1<<3)));                     \
        RCC->LSCTRL |= (1<<5);                              \
        ModuleIrqRemoval(BLE_SLP_IRQn);                     \
        ModuleIrqRegister(BLE_SLP_IRQn, rwip_slp_isr);      \
        g_sch_prog_fifo_iar_branch = false;                 \
        ModuleIrqRegister(BLE_FIFO_IRQn, rwip_fifo_isr);  \
        rwip_prog_delay                 = 3;                \
		rwip_init(RESET_NO_ERROR);                          \
}while(0)
#else //defined (MCU_LSC_SEL_LSI)
/* Select the LSI as Low speed clock source, don't change the LSC after this*/
#define NS_BLE_STACK_INIT()                                 \
do{                                                         \
        REG32(0x40007030)       = 0;                        \
		g_lsi_1_syscle_cnt_value = calib_lsi_clk();         \
		g_modem_flag            = MODEM_CHIP;               \
		g_app_init_branch       = false;                    \
		g_clock_src             = HSI_CLK;                  \
		g_system_hsi_clk        = HSI_VALUE;                \
		g_cal_hsi_cnt_value     = HSI_64M_VAL;              \
		g_delay_div_param       = 110;                      \
		g_delay_us_div_param    = 11;                       \
        llm_local_le_feats.feats[0] = 0xff;                 \
        llm_local_le_feats.feats[1] = 0xff;                 \
        llm_local_le_feats.feats[2] = 0xff;                 \
        llm_local_le_feats.feats[3] = 0x07;                 \
        llm_local_le_feats.feats[4] = 0x00;                 \
        llm_local_le_feats.feats[5] = 0x00;                 \
        llm_local_le_feats.feats[6] = 0x00;                 \
        llm_local_le_feats.feats[7] = 0x00;                 \
		REG32(0x200000dc) =(uint32_t) prf_add_profile;      \
		REG32(0x200000e0) =(uint32_t) prf_get_task_from_id; \
		REG32(0x200000e4) =(uint32_t) prf_get_id_from_task; \
        ModuleIrqRemoval(BLE_SLP_IRQn);                     \
        ModuleIrqRegister(BLE_SLP_IRQn, rwip_slp_isr);      \
        g_sch_prog_fifo_iar_branch = false;                 \
        ModuleIrqRegister(BLE_FIFO_IRQn, rwip_fifo_isr);  \
        rwip_prog_delay                 = 3;                \
		rwip_init(RESET_NO_ERROR);                          \
}while(0)
#endif
/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/


/* Public function prototypes -----------------------------------------------*/



#ifdef __cplusplus
}
#endif

#endif /* __BLE_STACK_COMMON_H_*/


