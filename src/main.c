//#include "../examples/CWTransmitter/CWTransmitter.c"
//#include "../examples/OOKTransmitter/OOKTransmitter.c"
//#include "../examples/AFSKTransmitter/AFSKTransmitter.c"
//#include "../examples/AFSKAprsTransmitter/AFSKAprsTransmitter.c"
//#include "../examples/MelodyPlayer/MelodyPlayer.c"
//#include "../examples/NewYearsBalloon/NewYearsBalloon.c"
#include "../examples/FoxTransmitter/FoxTransmitter.c"
//#include "../examples/FoxAprsTransmitter/FoxAprsTransmitter.c"
//#include "../examples/FoxConstantTransmitter/FoxConstantTransmitter.c"


// My patches in stm8s_it.h file requires empty implementations for some interrupts.
#include <drivers/interrupts_default.h>
