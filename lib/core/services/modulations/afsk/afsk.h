bool afsk_tone(uint16_t freqHz, unsigned long durationUs);

bool afsk_send_aprs_byte(char byte);

/*
 * Sends the space tone (2200Hz, typically a binary 0)
 */
bool afsk_aprs_send_space();

/*
 * Sends the mark tone (1200Hz, typically a binary 1)
 */
bool afsk_aprs_send_mark();
