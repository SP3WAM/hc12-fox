#ifndef SERVICES_MODULATIONS_CW_CW_RX_H
#define SERVICES_MODULATIONS_CW_CW_RX_H

#include <stdint.h>

#define cw_stop_rx() si4438_enter_ready_state()

#define cw_start_rx(channel) si4438_enter_rx_state(channel)

bool cw_init_rx();

#endif
