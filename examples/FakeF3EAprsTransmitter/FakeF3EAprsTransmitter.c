#include <Arduino.h>
#include "drivers/si4438.h"
#include "services/fake_f3e.h"

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
    Serial_print_s("Si4438 setting fake F3E mode...");
    if(fake_f3e_init_tx_direct_sync_2fsk() == false)
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
    fake_f3e_start_tx(0);

    // send some APRS bytes during some period (~200ms)
    unsigned long startMillis = millis();
    uint8_t q = 0;
    do
    {
        fake_f3e_send_aprs_byte(q);
        q++;
    } 
    while (millis() - startMillis < 200); 

    // stop transmition
    fake_f3e_stop_tx();
    
    // wait until the whole second passes
    delay(800);
}
