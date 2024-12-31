
// The channel 0 - defined in the radio_config_si4438_432_500.h file - has the freq 432.500 MHz. 
// Channel step is there defined as 12.5 kHz, so this gives us the following channels:

#define CHANNEL_APRS    0 // 432.500 MHz for APRS service on 70cm band
#define CHANNEL_433_500 80 // one of the direct channels on 70cm band
#define CHANNEL_FOX_0 128 // 434.100 MHz this is the base channel of all foxes (see correct radio_config_Si4438_xxx_xxx.h)
#define CHANNEL_FOX_1 132 // 434.150 MHz fox channels are seperated by 50 kHz
#define CHANNEL_FOX_2 136 // 434.200 MHz
#define CHANNEL_FOX_3 140 // 434.250 MHz
#define CHANNEL_FOX_4 144 // 434.300 MHz
#define CHANNEL_FOX_5 148 // 434.350 MHz
#define CHANNEL_FOX_6 152 // 434.400 MHz
#define CHANNEL_FOX_7 156 // 434.450 MHz
#define CHANNEL_FOX_8 160 // 434.500 MHz
#define CHANNEL_FOX_9 164 // 434.550 MHz this is the last possible channel according to IARU Region 1 UHF band plan
