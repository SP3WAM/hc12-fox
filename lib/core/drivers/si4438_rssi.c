#include <Arduino.h>
#include "si4438_rssi.h"
#include "si4438.h"

// Uncomment below line to have more debugs around RSSI calculations
#define DEBUG_RSSI

void get_average_rssi(uint8_t span_millis, uint8_t samples_count, average_rssi* result)
{
    #ifdef DEBUG_RSSI
    Serial_println_s("D get_average_rssi begin");
    #endif

    uint32_t rssiSumm = 0;
    uint32_t rssiSqSumm = 0;
    for(uint8_t q = 0 ; q < samples_count; q++)
    {
        uint8_t rssi;
        si4438_get_rssi(&rssi);

        rssiSumm += rssi;
        rssiSqSumm += ((uint16_t)rssi) * ((uint16_t)rssi);

        #ifdef DEBUG_RSSI
        Serial_print_s("D RSSI is ");
        Serial_println_i(rssi);
        #endif

        delay(span_millis);
    }
    uint8_t averageRssi = rssiSumm / samples_count;
    // https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Na%C3%AFve_algorithm
    uint16_t variance = (rssiSqSumm - rssiSumm * rssiSumm / samples_count ) / (samples_count - 1);
    uint8_t deviation = sqrt(variance);

    #ifdef DEBUG_RSSI
    Serial_print_s("D avgRSSI= ");
    Serial_print_i(averageRssi);
    Serial_print_s(" variance= ");
    Serial_print_i(variance);
    Serial_print_s(" deviation= ");
    Serial_println_i(deviation);
    #endif

    #ifdef DEBUG_RSSI
    Serial_println_s("D get_average_rssi end");
    #endif

    result->rssi = averageRssi;
    result->deviation = deviation;
}

uint8_t sqrt(uint16_t value)
{
    for(uint16_t q = 0; q < 20 ; q ++)
    {
        uint16_t square = q * q;
        if(square >= value)
        {
            return q;
        }
    }
    
    return 20;
}
