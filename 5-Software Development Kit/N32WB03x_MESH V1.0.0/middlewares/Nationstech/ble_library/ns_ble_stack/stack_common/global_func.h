

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "n32wb03x.h"
#include "global_var.h"
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



//extern uint32_t ModuleIrqRegister(IRQn_Type irqn,IRQ_HANNDLE_FUN fun);
//extern uint32_t ModuleIrqRemoval(IRQn_Type irqn);
extern void rwip_slp_isr(void);

extern void rf_tx_power_set(rf_tx_power_t pwr);
#endif    //_GLOBAL_FUNC_H_
