#include <Arduino.h>
#include "drivers/si4438.h"

void display_device_info()
{
    Serial_print_s("Si4438 getting func info... ");
    char info[6];
    if(si4438_get_func_info(info) == false)
    {
        Serial_println_s("failed");
    }
    else
    {
        char valueAsHexString[3];
        for(uint8_t q = 0 ; q < 6 ; q++)
        {
            sprintf(valueAsHexString, "%02X", info[q]);
            Serial_print_s("0x");
            Serial_print_s(valueAsHexString);
            Serial_print_s(" ");
        }
        Serial_println_s("");
    }
}