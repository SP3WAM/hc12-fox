#ifndef DRIVERS_SI4438_RSSI_H
#define DRIVERS_SI4438_RSSI_H

#include <stdint.h>

typedef struct
{
    uint8_t rssi;
    uint8_t deviation;
} average_rssi;


void si4438_get_average_rssi(uint8_t span_millis, uint8_t samples_count, average_rssi* result);

#endif