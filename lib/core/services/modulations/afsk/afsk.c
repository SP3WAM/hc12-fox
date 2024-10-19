#include <Arduino.h>
#include "afsk.h"

bool afsk_tone(uint16_t freqHz, unsigned long durationUs)
{
    unsigned long delayUs = 1000000L / freqHz / 2;
    unsigned long start = micros();

    do
    {
        digitalWrite(PB4, HIGH);
        delayMicroseconds(delayUs);
        digitalWrite(PB4, LOW);
        delayMicroseconds(delayUs);
    }
    while(micros() - start < durationUs);
}

bool afsk_send_aprs_byte(char byte)
{
    for(int q = 0 ; q < 8 ; q++)
    {
        if(byte & 0x01 == 0x01)
        {
            afsk_tone(1200, 833);
        }
        else
        {
            afsk_tone(2200, 833);
        }
        byte >> 1;
    }
}