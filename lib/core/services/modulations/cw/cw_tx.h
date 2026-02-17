#ifndef SERVICES_MODULATIONS_CW_CW_TX_H
#define SERVICES_MODULATIONS_CW_CW_TX_H

#include <stdint.h>

#define cw_stop_tx() si4438_enter_ready_state()

/*
 * Starts the CW transmition in specific channel. 
 */
#define cw_start_tx(channel) si4438_enter_tx_state(channel)

bool cw_init_tx();

#endif