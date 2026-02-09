#ifndef DRIVERS_SI4438_RSSI_H
#define DRIVERS_SI4438_RSSI_H

#include <stdint.h>

typedef struct
{
    uint8_t rssi;
    uint8_t deviation;
} average_rssi;


void get_average_rssi(uint8_t span_millis, uint8_t samples_count, average_rssi* result);

uint8_t sqrt(uint16_t value);

#endif