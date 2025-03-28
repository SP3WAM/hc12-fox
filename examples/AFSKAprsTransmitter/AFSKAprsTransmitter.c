#include <Arduino.h>
#include <drivers/si4438.h>
#include <services/modulations/fsk/fsk.h>
#include <services/modulations/afsk/afsk.h>
#include <radio_config_channels.h>

// Basic communication channels are defined in radio_config_channels.h
//#define COMMUNICATION_CHANNEL CHANNEL_FOX_0
#define COMMUNICATION_CHANNEL CHANNEL_APRS

#define lo8(x) ((x)&0xff)
#define hi8(x) ((x)>>8)

bool chipConnected = false;
unsigned long lastTxStartMillis = 0;
bool isTx = false;

uint16_t crc_ccitt_update(uint16_t crc, uint8_t data);

void setup()
{
    delay(3000);

    Serial_begin(115200);
    si4438_init_hw();
    delay(1000);

    // at first check if the hardware is connected
    Serial_print_s("Si4438 checking hardware...");
    chipConnected = si4438_is_chip_connected();
    if(chipConnected == false)
    {
        Serial_println_s(" failed");
    }
    Serial_println_s(" OK");

    // sending startup config is mandatory (especially POWER_UP which should 
    // be the first command of this config) 
    Serial_print_s("Si4438 apply startup config... ");
    if(si4438_apply_startup_config() == false)
    {
        Serial_println_s(" failed");
    }
    Serial_println_s(" OK");

    // configure the TX power
    Serial_print_s("Si4438 setting TX power...");
    if(si4438_set_tx_power(SI4438_MAX_TX_POWER) == false)
    {
        Serial_println_s(" failed");
    }
    Serial_println_s(" OK");

    // Init fake F3E transmission mode
    Serial_print_s("Si4438 setting FSK mode...");
    if(fsk_init_tx_direct_sync_2fsk() == false)
    {
        Serial_println_s(" failed");
    }
    Serial_println_s(" OK");
}

void loop()
{
    // Enter the Tx state on channel 0
    fsk_start_tx(COMMUNICATION_CHANNEL);

    // 0b0CRRSSSS
    //    C - command/response bit
    //   RR - reserved, should be 0b11
    // SSSS - four bits of SSID (Sub Station ID)
    // 1. Call sign must always be 6 bytes long. If it is shorter then padd it with spaces characters
    // 2. Modern digipiters doesn't respond to destination SSID different than 0 (doc/aprs/wb2osz/Understanding-APRS-Packets.pdf page 6)
    // 3. Destination Adress in ham APRS is more like source system type or source device id. It is not a call sign of destination station! (doc/aprs/wb2osz/Understanding-APRS-Packets.pdf page 5)
    // 4. For tests purposes can use APZxxx (xxx = <000,999>) as destination address (doc/aprs/aprs.org/tocalls.txt). SSID is not used.
    // SOURCE last byte:
    // 0b00110000; C=0, RR=11, SSID=0,  ASCII '0' no icon;
    // 0b00110001; C=0, RR=11, SSID=1,  ASCII '1' ambulance
    // 0b00110010; C=0, RR=11, SSID=2,  ASCII '2' bus
    // 0b00110011; C=0, RR=11, SSID=3,  ASCII '3' fire track
    // 0b00110100; C=0, RR=11, SSID=4,  ASCII '4' bicycle
    // 0b00110101; C=0, RR=11, SSID=5,  ASCII '5' yacht
    // 0b00110110; C=0, RR=11, SSID=6,  ASCII '6' helicopter
    // 0b00110111; C=0, RR=11, SSID=7,  ASCII '7' small aircratf
    // 0b00111000; C=0, RR=11, SSID=8,  ASCII '8' ship
    // 0b00111001; C=0, RR=11, SSID=9,  ASCII '9' car
    // 0b00111010; C=0, RR=11, SSID=10, ASCII ':' motorcycle
    // 0b00111011; C=0, RR=11, SSID=11, ASCII ';' balloon
    // 0b00111100; C=0, RR=11, SSID=12, ASCII '<' jeep
    // 0b00111101; C=0, RR=11, SSID=13, ASCII '=' recreational vehicle
    // 0b00111110; C=0, RR=11, SSID=14, ASCII '>' truck
    // 0b00111111; C=0, RR=11, SSID=15, ASCII '?' van

    uint8_t lastAddressIndex = 20;
    char minimalFrame[] = "APZ000 SP3WAM0WIDE2 1\x03\xF0:SP3IZN   :Co jest?##";
    uint8_t frameLength = sizeof(minimalFrame);

    // shift address bytes one bit to the left
    for(uint8_t q = 0 ; q <= lastAddressIndex ; q ++)
    {
        minimalFrame[q] = minimalFrame[q] << 1;
    }
    // set the LSB of the last address byte
    minimalFrame[lastAddressIndex] |= 0x01;

    // Frame Check Sequence - CRC-16-CCITT (0xFFFF)
    uint16_t crc = 0xFFFF;
    for(uint16_t i = 0; i < frameLength - 2; i++)
    {
        crc = crc_ccitt_update(crc, minimalFrame[i]);
    }
    crc = ~crc;                              // flip the bits
    minimalFrame[frameLength - 2] = crc & 0xFF;           // FCS is sent low-byte first
    minimalFrame[frameLength - 1] = (crc >> 8) & 0xFF;

    // init APRS transmition
    afsk_send_aprs_init();
    // send APRS packet
    afsk_send_aprs_packet(minimalFrame, frameLength);

    // stop transmition
    fsk_stop_tx();
    
    // wait a bit
    delay(35000);
}

uint16_t crc_ccitt_update(uint16_t crc, uint8_t data)
{
	data ^= lo8 (crc);
	data ^= data << 4;
	
	return ((((uint16_t)data << 8) | hi8 (crc)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
}

INTERRUPT_HANDLER(AWU_IRQHandler, ITC_IRQ_AWU)
{

}