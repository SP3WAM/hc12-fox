#include <Arduino.h>
#include "afsk.h"
#include "../fsk/fsk.h"

#define MICROS_DURATIN_US 43
#define AFSK1200_SPACE_HALF_DURATION_US (227 - 33)
#define AFSK1200_SPACE_HALF_SHORT_DURATION_US (227 - 76 - 33)
#define AFSK1200_MARK_HALF_DURATION_US (417 - 37)

uint8_t lastSentSymbol = 1; // meaning MARK as default


void afsk_send_aprs_byte_no_bit_stuffing(char byte);
void afsk_aprs_send_space();
void afsk_aprs_send_mark();

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

void afsk_send_aprs_init()
{
    lastSentSymbol = 0;
}

void afsk_send_aprs_packet(char* packet, uint8_t length)
{
    for(uint8_t q = 0 ; q < 40 ; q++)
    {
        afsk_send_aprs_byte_no_bit_stuffing(0xE7);
    }

    for(uint8_t q = 0 ; q < length ; q++)
    {
        afsk_send_aprs_byte_no_bit_stuffing(packet[q]);
    }

    for(uint8_t q = 0 ; q < 40 ; q++)
    {
        afsk_send_aprs_byte_no_bit_stuffing(0xE7);
    }
}

void afsk_send_aprs_byte_no_bit_stuffing(char byte)
{
    for(int q = 0 ; q < 8 ; q++)
    {
        if(byte & 0x01)
        {
            // bit 1: repeat last sent symbol
            if(lastSentSymbol == 1)
            {
                afsk_aprs_send_mark();
            }
            else
            {
                afsk_aprs_send_space();
            }
        }
        else
        {
            // bit 0: switch to the other symbol
            if(lastSentSymbol == 1)
            {
                afsk_aprs_send_space();
                lastSentSymbol = 0;
            }
            else
            {
                afsk_aprs_send_mark();
                lastSentSymbol = 1;
            }
        }
        byte >>= 1;
    }
}

/*
 * Sends the space tone (2200Hz, typically a binary 0 if not using extra encoding, i.e. NRZI)
 */
inline void afsk_aprs_send_space()
{
    fsk_tx_direct_bit_high();
    delayMicroseconds(AFSK1200_SPACE_HALF_DURATION_US);
    fsk_tx_direct_bit_low();
    delayMicroseconds(AFSK1200_SPACE_HALF_DURATION_US);
    fsk_tx_direct_bit_high();
    delayMicroseconds(AFSK1200_SPACE_HALF_DURATION_US);
    fsk_tx_direct_bit_low();
    // total duration of this bit must be 1/1200 sec (833us), but we are sending two periods 
    // of tone 2200Hz (which takes 909us, so a bit to much),
    // so the last half needs to be shortened around 76us)
    delayMicroseconds(AFSK1200_SPACE_HALF_SHORT_DURATION_US);
}

/*
 * Sends the mark tone (1200Hz, typically a binary 1 if not using extra encoding, i.e. NRZI)
 */
inline void afsk_aprs_send_mark()
{
    fsk_tx_direct_bit_high();
    delayMicroseconds(AFSK1200_MARK_HALF_DURATION_US);
    fsk_tx_direct_bit_low();
    delayMicroseconds(AFSK1200_MARK_HALF_DURATION_US);
}