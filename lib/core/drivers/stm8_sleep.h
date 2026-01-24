#ifndef DRIVERS_STM8_SLEEP_H
#define DRIVERS_STM8_SLEEP_H

#include <stdint.h>
#include <stm8s.h>

#define AWU_IRQHandler_DEFINED

#define STM8_S_SLEEP_250_MILLISEC() stm8s_sleep(10, 62)
#define STM8_S_SLEEP_500_MILLISEC() stm8s_sleep(11, 62)
#define STM8_S_SLEEP_2_25_SEC() stm8s_sleep(14, 28)
#define STM8_S_SLEEP_5_SEC() stm8s_sleep(14, 62)
#define STM8_S_SLEEP_20_SEC() stm8s_sleep(15, 41)

void stm8s_sleep(uint8_t tbr, uint8_t apr);

#endif