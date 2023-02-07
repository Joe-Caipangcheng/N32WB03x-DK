#ifndef MESH_TOOLCHAIN_H__
#define MESH_TOOLCHAIN_H__






#include "n32wb03x.h"


#define _DISABLE_IRQS(ui32IntStatus)        \
do{                                         \
    ui32IntStatus = __get_PRIMASK();        \
    __set_PRIMASK(1);                       \
}while(0);                                  

#define _ENABLE_IRQS(ui32IntStatus)         \
do{                                         \
    __set_PRIMASK(ui32IntStatus);           \
}while(0);                                  



typedef void (*IRQ_HANDLE_FUN)(void);
extern void ModuleIrqRemoval(IRQn_Type irqn);
extern uint32_t ModuleIrqRegister(IRQn_Type irqn, IRQ_HANDLE_FUN fun);







#endif /* TOOLCHAIN_H__ */
