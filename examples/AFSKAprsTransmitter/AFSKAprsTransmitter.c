#include <Arduino.h>
#include <drivers/si4438.h>
#include <services/modulations/fsk/fsk.h>
#include <services/modulations/afsk/afsk.h>

bool chipConnected = false;
unsigned long lastTxStartMillis = 0;
bool isTx = false;

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
    // RS41 sonde sends frame with max 320 bytes
    // The frame which it leads is 320* bytes long and thus counts just over 533 ms


    // Enter the Tx state on channel 0
    fsk_start_tx(0);

    // char minimalFrame[] = {'S', 'P', '3', 'W', 'A', 'M', ' ', '1', 'S', 'P', '3', 'W', 'A', 'M', ' ', '1', 0x03, 0xF0, 'H', 'e', 'l', 'l', 'o', '#', '#'};
    
    // // sent some flags to indicate incoming packet
    // for(uint8_t q = 0 ; q < 10 ; q++)
    // {
    //     afsk_send_aprs_byte(0x7F);
    // }
    // // send the packet itself
    // for(uint8_t q = 0; q < 25; q++)
    // {
    //     afsk_send_aprs_byte(minimalFrame[q]);
    // }
    // // send some flags at end
    // for(uint8_t q = 0 ; q < 10 ; q++)
    // {
    //     afsk_send_aprs_byte(0x7F);
    // }

    // // stop transmition
    // fsk_stop_tx();
    
    // // wait until the whole second passes
    // delay(800);

    // send 0xFF (ones) for 10 seconds
    for(uint16_t q = 0; q < 750; q++)
    {
        afsk_send_aprs_byte(0xFF);
    }

    // send 0x00 (zeros) for 10 seconds
    for(uint16_t q = 0; q < 750; q++)
    {
        afsk_send_aprs_byte(0x00);
    }

    //afsk_tone_half_duration_us(417, 5000000);
    //afsk_tone_half_duration_us(227, 5000000);
}


INTERRUPT_HANDLER(AWU_IRQHandler, 1)
{

}