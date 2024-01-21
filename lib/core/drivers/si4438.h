#include <stdbool.h>
#include <stdint.h>

#define SI4438_MAX_TX_POWER 127
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

bool si4438_enter_ready_state();

bool si4438_enter_tx_state();

bool si4438_get_part_info(char* part_info);

bool si4438_get_func_info(uint8_t* func_info);
