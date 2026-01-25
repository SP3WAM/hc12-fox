#ifndef SERVICES_MODULATIONS_FSK_FSK_H
#define SERVICES_MODULATIONS_FSK_FSK_H

#include <stdbool.h>
#include "../../../drivers/si4438.h"

/*
 * Starts the FSK transmition on specific channel.
 */
#define fsk_start_tx(channel) si4438_enter_tx_state(channel)

/*
 * Stops the FSK transmition.
 */
#define fsk_stop_tx() si4438_enter_ready_state()

/*
 * Transmits bit HIGH.
 */
#define fsk_tx_direct_bit_high() digitalWrite(PB4, HIGH)

/*
 * Transmits bit LOW.
 */
#define fsk_tx_direct_bit_low() digitalWrite(PB4, LOW)

#endif