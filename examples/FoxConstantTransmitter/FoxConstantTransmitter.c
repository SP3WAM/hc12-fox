#include <Arduino.h>
#include <drivers/si4438.h>
#include <services/modulations/fsk/fsk.h>
#include <services/modulations/afsk/afsk.h>
#include <radio_config_channels.h>

/*
 * FOX CONFIGURATION SECTION BEGIN
 */

// Basic communication channels are defined in radio_config_channels.h
#define COMMUNICATION_CHANNEL CHANNEL_FOX_2

// Basic transmission power; allows to locate the fox from far distance
#define TRANSMISSION_POWER SI4438_17DBM_TX_POWER
/*
 * FOX CONFIGURATION SECTION END
*/

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
    if(si4438_is_chip_connected() == false)
    {
        Serial_println_s(" failed");
    }
    else
    {
        Serial_println_s(" OK");   
    }

    // sending startup config is mandatory (especially POWER_UP which should 
    // be the first command of this config) 
    Serial_print_s("Si4438 apply startup config... ");
    if(si4438_apply_startup_config() == false)
    {
        Serial_println_s(" failed");
    }
    else
    {
        Serial_println_s(" OK");
    }

    // configure the TX power
    Serial_print_s("Si4438 setting TX power...");
    if(si4438_set_tx_power(TRANSMISSION_POWER) == false)
    {
        Serial_println_s(" failed");
    }
    else
    {
        Serial_println_s(" OK");
    }

    // Init fake F3E transmission mode
    Serial_print_s("Si4438 setting FSK mode...");
    if(fsk_init_tx_direct_sync_2gfsk() == false)
    {
        Serial_println_s(" failed");
    }
    else
    {
        Serial_println_s(" OK");
    }
}

void loop()
{
    fsk_start_tx(COMMUNICATION_CHANNEL);

    while(true)
    {
        afsk_tone(700, 500000ul);
        delay(250);
        afsk_tone(800, 500000ul);
        delay(250);
        afsk_tone(900, 500000ul);
        delay(500);
    } 
}

/*
 * Empty interrupt handler.
 */
INTERRUPT_HANDLER(AWU_IRQHandler, 1)
{

}
