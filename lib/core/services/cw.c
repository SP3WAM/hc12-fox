
#include <Arduino.h>
#include <stddef.h>
#include "cw.h"
#include "../drivers/si4438.h"

bool cw_init_tx_pseudo()
{
    // configure GPIO
    uint8_t gpioCmd[8];
    gpioCmd[0] = SI4438_CMD_GPIO_PIN_CFG;
    gpioCmd[1] = 0x00; // GPIO0
    gpioCmd[2] = 0x00; // GPIO1
    gpioCmd[3] = 0b0000011; // GPIO2: CMOS output driven high, pull up disabled
    gpioCmd[4] = 0b0000010; // GPIO3: CMOS output driven low, pull up disabled
    gpioCmd[5] = 0x00; // NIRQ
    gpioCmd[6] = 0x00; // SDO
    gpioCmd[7] = 0x00; // GEN_CONFIG

    if(doAPI(gpioCmd, sizeof(gpioCmd), NULL, 0) == false)
    {
        return false;
    }

    // configure CW Tx mode
    //   TX_DIRECT_MODE_TYPE[0] = 0b0;   not used as we have PSEUDO mode
    // TX_DIRECT_MODE_GPIO[1:0] = 0b00;  not used as we have PSEUDO mode
    //          MOD_SOURCE[1:0] = 0b10;  PSEUDO. The modulation is sourced from the internal pseudo-random generator.
    //            MOD_TYPE[2:0] = 0b000; CW
    uint8_t value = 0b00010000;
    if(setProperty(SI44338_PROPERTY_MODEM_MOD_TYPE, value) == false)
    {
        return false;
    }

    return cw_stop_tx();
}

bool cw_init_tx_direct()
{
    // configure GPIO of Si44xx
    uint8_t gpioCmd[8];
    gpioCmd[0] = SI4438_CMD_GPIO_PIN_CFG;
    gpioCmd[1] = 0b00000100; // GPIO0: INPUT Pin is configured as CMOS input. Pull Up disabled.
    gpioCmd[2] = 0b00010000; // GPIO1: TX_DATA_CLK Outputs the TX Data Clock signal. Pull Up disabled.
    gpioCmd[3] = 0b00000011; // GPIO2: CMOS output driven high, pull up disabled. Sets the RF switch into TX mode.
    gpioCmd[4] = 0b00000010; // GPIO3: CMOS output driven low, pull up disabled. Sets the RF switch into TX mode.
    gpioCmd[5] = 0x00; // NIRQ
    gpioCmd[6] = 0x00; // SDO
    gpioCmd[7] = 0x00; // GEN_CONFIG

    if(doAPI(gpioCmd, sizeof(gpioCmd), NULL, 0) == false)
    {
        return false;
    }

    // configure GPIO of STM8
    // GPIO1 -> PC3 as input pull up
    pinMode(PC3, INPUT);
    digitalWrite(PC3, HIGH);
    // GPIO0 <- PB4 as output always high (CW generated constantly)
    pinMode(PB4, OUTPUT);
    // digitalWrite(PB4, LOW);
    digitalWrite(PB4, HIGH);

    // configure CW Tx Synchronous Direct mode
    //   TX_DIRECT_MODE_TYPE[0] = 0b0;   Direct mode operates in synchronous mode, applies to TX only.
    // TX_DIRECT_MODE_GPIO[1:0] = 0b00;  TX direct mode uses GPIO0 as data source.
    //          MOD_SOURCE[1:0] = 0b01;  The modulation is sourced in real-time from a GPIO pin
    //            MOD_TYPE[2:0] = 0b000; CW
    uint8_t value = 0b00001000;
    if(setProperty(SI44338_PROPERTY_MODEM_MOD_TYPE, value) == false)
    {
        return false;
    }

    return cw_stop_tx();
}

bool cw_start_tx()
{
    return si4438_enter_tx_state();
}

bool cw_stop_tx()
{
    return si4438_enter_ready_state();
}