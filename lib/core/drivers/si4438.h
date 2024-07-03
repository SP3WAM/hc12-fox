#include <stdbool.h>
#include <stdint.h>

#define SI4438_MAX_TX_POWER 127
#define SI4438_10DBM_TX_POWER  20
#define SI4438_MIN_TX_POWER 0

#define SI4438_CMD_PART_INFO 0x01
#define SI4438_CMD_POWER_UP 0x02
#define SI4438_CMD_FUNC_INFO 0x10
#define SI4438_CMD_SET_PROPERTY 0x11
#define SI4438_CMD_GET_PROPERTY 0x12
#define SI4438_CMD_GPIO_PIN_CFG 0x13
#define SI4438_CMD_START_TX 0x31
#define SI4438_CMD_CHANGE_STATE 0x34
#define SI4438_CMD_READ_CMD_BUFF 0x44

#define SI4438_PROPERTY_PA_PWR_LVL 0x2201
#define SI44338_PROPERTY_MODEM_MOD_TYPE 0x2000

bool si4438_init_hw();

bool si4438_is_chip_connected();

bool doAPI(uint8_t* data, uint8_t len, uint8_t* out, uint8_t outLen);

bool si4438_get_property(uint16_t prop, uint8_t* value);

bool setProperty(uint16_t prop, uint8_t value);

bool si4438_apply_startup_config();

/*
 * PA Power Level Configuration.
 *
 * Adjusts the TX power level in fine resolution.
 * Range: 0-127
 */
bool si4438_set_tx_power(uint8_t pwr);

bool si4438_enter_sleep_state();

bool si4438_enter_ready_state();

bool si4438_enter_tx_state();

bool si4438_get_part_info(char* part_info);

bool si4438_get_func_info(uint8_t* func_info);

/*
 * Sets the base frequency.
 * Euqtion freq[Hz] = (fc_inte + fc_frac / 2^19) * Npresc * freq_xo / outdiv
 * where in SI4438 of HC-12 board:
 * Npresc = 2
 * freq_xo = 30000000 Hz
 * outdiv = 8
 * 
 * fc_inte and fc_frac must be calculated and written to the chip. The values may be found from the equation above by taking the minimal
 * values of fc_inte and fc_frac that solves the equation, however there is written in the datasheet of SI4438, that the value of fc_frac/2^19 
 * must be in range <1,2>, to have this possible the value of fc_inte
 * calculation must be 1 less than preliminary minimal values of fc_inte and fc_frac calculated at the first time.
 *
 * Hint: can use WDS3 software to check the calculations, by changing the base frequency only and comparing the output configuration 
 * header file.
 */
bool si4438_set_base_frequency(unsigned long freq_Hz);
