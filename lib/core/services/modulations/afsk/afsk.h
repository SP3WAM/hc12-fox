bool afsk_tone(uint16_t freqHz, unsigned long durationUs);

/*
 * Prepares the state machine to send APRS packet
 */
void afsk_send_aprs_init();

/*
 * Sends the APRS packet into the transmisison medium.
 * The following aspects are taken into account:
 * 1. sending the flags before packet
 * 2. sending the packet itself
 *  a. bit stuffing
 *  b. NRZI encoding
 *  c. sending mark or space symbols accordingly
 * 3. sending the flags after the packet
 */
void afsk_send_aprs_packet(char* packet, uint8_t length);
