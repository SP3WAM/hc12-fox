#ifndef DRIVERS_INTERRUPTS_DEFAULT_H
#define DRIVERS_INTERRUPTS_DEFAULT_H

// My patches in stm8s_it.h file requires empty implementations for some interrupts,
// but only in case if they are not defined already by fox functionality.

#include <stm8s_it.h>

#ifndef AWU_IRQHandler_DEFINED
INTERRUPT_HANDLER(AWU_IRQHandler, 1)
{
    // do nothing
}
#endif

#ifndef TIM2_UPD_OVF_BRK_IRQHandler_DEFINED  
INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, ITC_IRQ_TIM1_OVF)
{
    // do nothing
}
#endif

#endif