#include <Arduino.h>
#include "afsk_tone.h"
#include "../fsk/fsk.h"

#define MICROS_DURATIN_US 43

bool afsk_tone(uint16_t freqHz, unsigned long durationUs)
{
    unsigned long delayUs = 1000000L / freqHz / 2;
    unsigned long secondDelayUs = delayUs - MICROS_DURATIN_US;
    unsigned long start = micros();

    do
    {
        fsk_tx_direct_bit_high();
        delayMicroseconds(delayUs);
        fsk_tx_direct_bit_low();
        delayMicroseconds(secondDelayUs);
    }
    while(micros() - start < durationUs);
}

