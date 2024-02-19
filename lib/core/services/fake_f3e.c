#include <stddef.h>
#include "fake_f3e.h"
#include "../drivers/si4438.h"

bool fake_f3e_init_tx()
{
    // configure GPIOs
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

    // configure 2FSK Tx mode
    //      TX_DIRECT_MODE_TYPE = 0b0;   not used as we have PSEUDO mode
    // TX_DIRECT_MODE_GPIO[1:0] = 0b00;  not used as we have PSEUDO mode
    //          MOD_SOURCE[1:0] = 0b10;  PSEUDO. The modulation is sourced from the internal pseudo-random generator.
    //            MOD_TYPE[2:0] = 0b010; 2FSK
    uint8_t value = 0b00010010; 
    if(setProperty(SI44338_PROPERTY_MODEM_MOD_TYPE, value) == false)
    {
        return false;
    }

    return fake_f3e_stop_tx();
}

bool fake_f3e_start_tx()
{
    si4438_enter_tx_state();
}

bool fake_f3e_stop_tx()
{
    si4438_enter_ready_state();
}