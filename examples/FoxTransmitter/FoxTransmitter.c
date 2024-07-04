#include <Arduino.h>
#include "drivers/si4438.h"
#include "services/fake_f3e.h"

bool chipConnected = false;
unsigned long lastTxStartMillis = 0;
bool isTx = false;

#define DOT_DURATION_MILLIS  150
#define DASH_DURATION_MILLIS 3 * DOT_DURATION_MILLIS

void sendDot();
void sendDash();
void sendMOE();
void sendMOI();
void sendMOS();
void sendMOH();
void sendMO5();

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
    if(si4438_set_tx_power(SI4438_10DBM_TX_POWER) == false)
    {
        Serial_println_s(" failed");
    }
    else
    {
        Serial_println_s(" OK");
    }

    // Init fake F3E transmission mode
    Serial_print_s("Si4438 setting fake F3E mode...");
    if(fake_f3e_init_tx_direct_sync_2gfsk() == false)
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
    fake_f3e_start_tx(0); // channel 0: 434.100 MHz
    sendMOE();
    // fake_f3e_start_tx(4); // channel 4: 434.150 MHz
    // sendMOI();
    // fake_f3e_start_tx(8); // channel 8: 434.200 MHz
    // sendMOS();
    // fake_f3e_start_tx(12); // channel 12: 434.250 MHz
    // sendMOH();
    // fake_f3e_start_tx(16); // channel 16: 434.300 MHz
    // sendMO5();
    // fake_f3e_start_tx(20); // channel 20: 434.350 MHz
    // sendMO5();
    // fake_f3e_start_tx(24); // channel 24: 434.400 MHz
    // sendMO5();
    
    fake_f3e_stop_tx();
    si4438_enter_sleep_state();

    delay(5000);
}

void sendMOE()
{
    // send M
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(3 * DOT_DURATION_MILLIS);
    
    // send O
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(3 * DOT_DURATION_MILLIS);

    // send E
    sendDot();
    delay(7 * DOT_DURATION_MILLIS);
}

void sendMOI()
{
    // send M
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(3 * DOT_DURATION_MILLIS);
    
    // send O
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(3 * DOT_DURATION_MILLIS);

    // send I
    sendDot();
    delay(DOT_DURATION_MILLIS);  
    sendDot();
    delay(7 * DOT_DURATION_MILLIS);
}

void sendMOS()
{
    // send M
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(3 * DOT_DURATION_MILLIS);
    
    // send O
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(3 * DOT_DURATION_MILLIS);

    // send S
    sendDot();
    delay(DOT_DURATION_MILLIS); 
    sendDot();
    delay(DOT_DURATION_MILLIS);  
    sendDot();
    delay(7 * DOT_DURATION_MILLIS);
}

void sendMOH()
{
    // send M
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(3 * DOT_DURATION_MILLIS);
    
    // send O
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(3 * DOT_DURATION_MILLIS);

    // send H
    sendDot();
    delay(DOT_DURATION_MILLIS); 
    sendDot();
    delay(DOT_DURATION_MILLIS); 
    sendDot();
    delay(DOT_DURATION_MILLIS);  
    sendDot();
    delay(7 * DOT_DURATION_MILLIS);
}

void sendMO5()
{
    // send M
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(3 * DOT_DURATION_MILLIS);
    
    // send O
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(3 * DOT_DURATION_MILLIS);

    // send 5
    sendDot();
    delay(DOT_DURATION_MILLIS); 
    sendDot();
    delay(DOT_DURATION_MILLIS); 
    sendDot();
    delay(DOT_DURATION_MILLIS); 
    sendDot();
    delay(DOT_DURATION_MILLIS);  
    sendDot();
    delay(7 * DOT_DURATION_MILLIS);
}

void inline sendDot()
{
    fake_f3e_tone(500, ((unsigned long)DOT_DURATION_MILLIS) * ((unsigned long)1000));
}

void inline sendDash()
{
    fake_f3e_tone(500, ((unsigned long)DASH_DURATION_MILLIS) * ((unsigned long)1000));
}