#include <Arduino.h>
#include "afsk.h"
#include "../fsk/fsk.h"

bool afsk_tone(uint16_t freqHz, unsigned long durationUs)
{
    unsigned long delayUs = 1000000L / freqHz / 2;
    unsigned long start = micros();

    do
    {
        fsk_tx_direct_bit_high();
        delayMicroseconds(delayUs);
        fsk_tx_direct_bit_low();
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