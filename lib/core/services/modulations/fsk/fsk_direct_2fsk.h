#ifndef SERVICES_MODULATIONS_FSK_FSK_DIRECT_2FSK_H
#define SERVICES_MODULATIONS_FSK_FSK_DIRECT_2FSK_H

#include "fsk.h"

/*
 * Initiates direct synchronised transmition 2FSK mode. 
 * In this mode the bits must be fed to the transceiver by driving its GPIO0 pin.
 * Modulation type is 2FSK, where two different frequencies are used so the generated stream will be hearable
 * on FM receiver.
 */
bool fsk_init_tx_direct_sync_2fsk();


#endif