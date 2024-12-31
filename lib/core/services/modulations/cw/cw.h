#include <stdbool.h>

bool cw_init_tx();

bool cw_init_rx();

/*
 * Starts the CW transmition in specific channel. 
 */
bool cw_start_tx(uint8_t channel);

bool cw_start_rx(uint8_t channel);

bool cw_stop_txrx();
