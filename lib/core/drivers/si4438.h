#include <stdbool.h>
#include <stdint.h>

#define SI4438_MIN_TX_POWER     0  // -35dBm (~0.0003mW)
#define SI4438_MAX_TX_POWER   127  //  20dBm (100mW)
#define SI4438_NEG35DBM_TX_POWER 0 // 0.0003mW
#define SI4438_NEG21DBM_TX_POWER 1 // 0.008mW
#define SI4438_NEG12DBM_TX_POWER 2 // 0.06mW
#define SI4438_NEG8DBM_TX_POWER 3 // 0.2mW
#define SI4438_00DBM_TX_POWER   6  // 1mW
#define SI4438_10DBM_TX_POWER  20  // 10mW
#define SI4438_13DBM_TX_POWER  28  // 20mW
#define SI4438_15DBM_TX_POWER  35  // 30mW
#define SI4438_16DBM_TX_POWER  40  // 40mW
#define SI4438_17DBM_TX_POWER  48  // 50mW
#define SI4438_20DBM_TX_POWER  SI4438_MAX_TX_POWER  // 100mW


#define SI4438_CMD_PART_INFO 0x01
#define SI4438_CMD_POWER_UP 0x02
#define SI4438_CMD_FUNC_INFO 0x10
#define SI4438_CMD_SET_PROPERTY 0x11
#define SI4438_CMD_GET_PROPERTY 0x12
#define SI4438_CMD_GPIO_PIN_CFG 0x13
#define SI4438_CMD_GET_MODEM_STATUS 0x22
#define SI4438_CMD_START_TX 0x31
#define SI4438_CMD_START_RX 0x32
#define SI4438_CMD_CHANGE_STATE 0x34
#define SI4438_CMD_READ_CMD_BUFF 0x44

#define SI4438_PROPERTY_PA_PWR_LVL 0x2201
#define SI44338_PROPERTY_MODEM_MOD_TYPE 0x2000
#define SI4438_PROPERTY_MODEM_RSSI_THRESH 0x204A

bool si4438_init_hw();

bool si4438_is_chip_connected();

bool si4438_doAPI(uint8_t* data, uint8_t len, uint8_t* out, uint8_t outLen);

bool si4438_get_property(uint16_t prop, uint8_t* value);

bool si4438_setProperty(uint16_t prop, uint8_t value);

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

/**
 * Starts the transmition on specific channel.
 * Channel range: 0 - 255
 * The transmition frequency is given by the following equation:
 * RF Freq.(MHz) + fhst x channel
 * where the parameters are configured in radio_config.h with the help of WDS3 software.
 */
bool si4438_enter_tx_state(uint8_t channel);

bool si4438_enter_rx_state(uint8_t channel);

bool si4438_get_part_info(char* part_info);

bool si4438_get_func_info(uint8_t* func_info);

/**
 * Gets the current RSSI as a value between 0 (the worst signal) and 255 (the best signal).
 * It is not a dBm meassure.
 */
bool si4438_get_rssi(uint8_t* rssi);
