#include <radio_config_Si4438_434_000_channels.h>
#include <drivers/si4438_power.h>

// Define your call sign here 
// default: SP3WAM
char CALL_SIGN[] = "... .--. ...-- .-- .- --";


// Define fox transmition channel
// default: CHANNEL_FOX_0
#define COMMUNICATION_CHANNEL CHANNEL_FOX_0


// Channel offset for bad clones of Si4438; added to the COMMUNICATION_CHANNEL defined above.
// It may be negative.
// default: 0
#define COMMUNICATION_CHANNEL_OFFSET 0


// Defines the additional SNR level (added to the current average noise level) to wake the fox up.
// Expressed in unit of 0.5 dB
// default: 12 (which is 6 dB)
#define RSSI_ADDITIONAL_TRESHOLD_SNR 12


// Basic transmission power; allows to locate the fox from far distance
// default: SI4438_17DBM_TX_POWER (-17 dBm 50 mW)
#define TRANSMISSION_POWER SI4438_17DBM_TX_POWER


// Nearby transmission power; helps to locate the fox from nearby
// default: SI4438_NEG21DBM_TX_POWER (which is -21 dBm 0.008 mW)
#define TRANSMISSION_NEARBY_POWER SI4438_NEG21DBM_TX_POWER

