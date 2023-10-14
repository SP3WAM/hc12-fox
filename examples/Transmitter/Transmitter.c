#include <Arduino.h>
#include "drivers/si4438.h"

bool chipConnected = false;

void setup()
{
    delay(1000);

    Serial_begin(115200);
    si4438_init_hw();
    delay(1000);

    chipConnected = si4438_is_chip_connected();
    if(chipConnected)
    {
        Serial_println_s("Si4438 chip connected.");
    }
    else
    {
        Serial_println_s("Si4438 chip not connected.");
    }
}

void loop()
{
    Serial_print_i(millis());
    Serial_println_s(" loop()");

    if(si4438_is_chip_connected())
    {
        Serial_println_s("Si4438 chip connected.");
    }
    else
    {
        Serial_println_s("Si4438 chip not connected.");
    }

    delay(1000);
}
