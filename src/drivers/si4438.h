#include <stdbool.h>


#define SI4438_MAX_TX_POWER 127
#define SI4438_MIN_TX_POWER 0


bool si4438_init_hw();

bool si4438_is_chip_connected();

/*
 * Power-up device and mode selection. Modes include operational function.
 *
 * Power-up the device with the specified function. Power-up is complete when the CTS bit is set. This command may take
 * longer to set the CTS bit than other commands. Without Power-up the chip will not accept any properties.
 */
bool si4438_power_up();

bool si4438_set_freq();

/*
 * PA Power Level Configuration.
 *
 * Adjusts the TX power level in fine resolution.
 * Range: 0-127
 */
bool si4438_set_tx_power(uint8_t pwr);

//bool si4438_init_cw();

//bool si4438_cw_on();

//bool si4438_cw_off();
