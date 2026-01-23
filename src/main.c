//#include "../examples/CWTransmitter/CWTransmitter.c"
//#include "../examples/OOKTransmitter/OOKTransmitter.c"
//#include "../examples/AFSKTransmitter/AFSKTransmitter.c"
//#include "../examples/AFSKAprsTransmitter/AFSKAprsTransmitter.c"
//#include "../examples/MelodyPlayer/MelodyPlayer.c"
//#include "../examples/NewYearsBalloon/NewYearsBalloon.c"
#include "../examples/FoxTransmitter/FoxTransmitter.c"
//#include "../examples/FoxAprsTransmitter/FoxAprsTransmitter.c"
//#include "../examples/FoxConstantTransmitter/FoxConstantTransmitter.c"



// My patches in stm8s_it.h file requires empty implementations for some interrupts,
// but only in case if they are not defined already by fox functionality.
#ifndef AWU_ISR_DEFINED
INTERRUPT_HANDLER(AWU_IRQHandler, 1)
{
    // do nothing
}
#endif

#ifndef TIM2_UPD_OVF_BRK_ISR_DEFINED
INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, ITC_IRQ_TIM1_OVF)
{
    // do nothing
}
#endif
