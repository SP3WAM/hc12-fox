#include <Arduino.h>
#include "drivers/si4438.h"

bool chipConnected = false;

void setup()
{
    delay(1000);

    Serial_begin(115200);
    si4438_init_hw();
    delay(1000);

    // at first check if the hardware is connected
    Serial_print_s("Checking Si4438 hardware...");
    chipConnected = si4438_is_chip_connected();
    if(chipConnected == false)
    {
        Serial_println_s(" failed");
        return;
    }
    Serial_println_s(" OK");

    // sending POWER_UP is mandatory otherwise the chip will not accepty any properties
    Serial_print_s("Sending Si4438 POWER_UP...");
    if(si4438_power_up() == false)
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
