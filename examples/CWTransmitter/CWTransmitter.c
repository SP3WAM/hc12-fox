#include <Arduino.h>
#include <drivers/si4438.h>
#include <services/modulations/cw/cw.h>
#include <services/terminal.h>

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

    // display device info
    //display_device_info();

    // configure the TX power
    Serial_print_s("Si4438 setting TX power...");
    if(si4438_set_tx_power(SI4438_MAX_TX_POWER) == false)
    {
        Serial_println_s(" failed");
    }
    Serial_println_s(" OK");

    // Init CW transmitter
    Serial_print_s("Si4438 setting CW mode...");
    if(cw_init_tx() == false)
    {
        Serial_println_s(" failed");
    }
    Serial_println_s(" OK");

    terminal_init();

    cw_start_tx(0);
}

void loop()
{
    terminal_loop();

    // WARNING: entering the TX state constantly has impact to the transmitter:
    // it generates something but it is not hearable in the receiver
    // cw_start_tx();
}

/*
 * Empty interrupt handler.
 */
INTERRUPT_HANDLER(AWU_IRQHandler, 1)
{

}
