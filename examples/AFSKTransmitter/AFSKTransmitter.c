#include <Arduino.h>
#include <drivers/si4438.h>
#include <services/modulations/fsk/fsk.h>
#include <services/modulations/afsk/afsk.h>
#include <radio_config_channels.h>

// Basic communication channels are defined in radio_config_channels.h
#define COMMUNICATION_CHANNEL CHANNEL_FOX_0

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
    if(si4438_set_tx_power(20) == false)
    {
        Serial_println_s(" failed");
    }
    Serial_println_s(" OK");

    // Init fake F3E transmission mode
    Serial_print_s("Si4438 setting FSK mode...");
    if(fsk_init_tx_direct_sync_2gfsk() == false)
    {
        Serial_println_s(" failed");
    }
    Serial_println_s(" OK");
}

void loop()
{
    fsk_start_tx(COMMUNICATION_CHANNEL);

    afsk_tone(1000, 2000000L);

    fsk_stop_tx();

    delay(2000);
}

/*
 * Empty interrupt handler.
 */
INTERRUPT_HANDLER(AWU_IRQHandler, 1)
{

}
