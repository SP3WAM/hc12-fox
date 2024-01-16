#include <Arduino.h>
#include "drivers/si4438.h"
#include "services/fake_f3e.h"
#include "services/terminal.h"

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
    if(fake_f3e_init_tx() == false)
    {
        Serial_println_s(" failed");
    }
    Serial_println_s(" OK");

    terminal_init();
}

void loop()
{
    terminal_loop();
    unsigned long now = millis();

    if(isTx == true && (now - lastTxStartMillis >= 1000))
    {
        // need to disable TX
        //Serial_println_s("Si4438 stop fake F3E");
        fake_f3e_stop_tx();
        isTx = false;
    }
    
    if(isTx == false && (now - lastTxStartMillis >= 2000))
    {
        // need to enable Tx
        //Serial_println_s("Si4438 start fake F3E");
        fake_f3e_start_tx();
        isTx = true;
        lastTxStartMillis = now;
    }
}
