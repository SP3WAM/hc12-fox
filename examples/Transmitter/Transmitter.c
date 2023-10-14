#include <Arduino.h>

void setup()
{
    Serial_begin(115200);
    delay(1000);
}

void loop()
{
    Serial_println_s("loop()");

    delay(1000);
}
