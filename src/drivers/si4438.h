#include <stdbool.h>

bool si4438_init_hw();

bool si4438_is_chip_connected();

bool si4438_set_freq();

/*
 * PA Power Level Configuration
 * Adjusts the TX power level in fine resolution.
 * Range: 0-127
 */
bool si4438_set_tx_power(uint8_t pwr);

//bool si4438_init_cw();

//bool si4438_cw_on();

//bool si4438_cw_off();
