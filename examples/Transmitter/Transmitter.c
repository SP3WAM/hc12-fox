#include <Arduino.h>
#include "drivers/si4438.h"

bool chipConnected = false;

void setup()
{
    delay(3000);

    Serial_begin(115200);
    si4438_init_hw();
    delay(1000);

    // at first check if the hardware is connected
    Serial_print_s("Si4438 cheking hardware...");
    chipConnected = si4438_is_chip_connected();
    if(chipConnected == false)
    {
        Serial_println_s(" failed");
        return;
    }
    Serial_println_s(" OK");

    // sending startup config is mandatory (especially POWER_UP which should 
    // be the first command of this config) 
    Serial_print_s("Si4438 apply startup config... ");
    if(si4438_apply_startup_config() == false)
    {
        Serial_println_s(" failed");
        return;
    }
    Serial_println_s(" OK");

    // configure the TX power
    Serial_print_s("Settingd Si4438 TX power...");
    if(si4438_set_tx_power(0x01) == false)
    {
        Serial_println_s(" failed");
        return;
    }
    Serial_println_s(" OK");
}

void loop()
{
    delay(1000);

    Serial_print_i(millis());
    Serial_println_s(" loop()");

    if(si4438_is_chip_connected())
    {
        Serial_println_s("Si4438 chip connected.");
    }
    else
    {
        Serial_println_s("Si4438 chip not connected.");
        return;
    }

    Serial_print_s("Settingd Si4438 TX power...");
    if(si4438_set_tx_power(0x10) == false)
    {
        Serial_println_s(" failed");
        return;
    }
    Serial_println_s(" OK");
}
