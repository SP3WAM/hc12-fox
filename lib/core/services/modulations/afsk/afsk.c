#include <Arduino.h>
#include "afsk.h"
#include "../fsk/fsk.h"

#define MICROS_DURATIN_US 43
#define AFSK1200_SPACE_HALF_DURATION_US (227 - 33)
#define AFSK1200_MARK_HALF_DURATION_US (417 - 37)

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

bool afsk_send_aprs_byte(char byte)
{
    for(int q = 0 ; q < 8 ; q++)
    {
        if(byte & 0x01)
        {
            // bit 1
            afsk_aprs_send_mark();
        }
        else
        {
            // bit 0
            afsk_aprs_send_space();
        }
        byte >>= 1;
    }
}

inline void afsk_aprs_send_space()
{
    fsk_tx_direct_bit_high();
    delayMicroseconds(AFSK1200_SPACE_HALF_DURATION_US);
    fsk_tx_direct_bit_low();
    delayMicroseconds(AFSK1200_SPACE_HALF_DURATION_US);
    fsk_tx_direct_bit_high();
    delayMicroseconds(AFSK1200_SPACE_HALF_DURATION_US);
    fsk_tx_direct_bit_low();
    delayMicroseconds(AFSK1200_SPACE_HALF_DURATION_US);
}

inline void afsk_aprs_send_mark()
{
    fsk_tx_direct_bit_high();
    delayMicroseconds(AFSK1200_MARK_HALF_DURATION_US);
    fsk_tx_direct_bit_low();
    delayMicroseconds(AFSK1200_MARK_HALF_DURATION_US);
}