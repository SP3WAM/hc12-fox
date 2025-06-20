#include <Arduino.h>
#include <drivers/si4438.h>
#include <services/modulations/fsk/fsk.h>
#include <services/modulations/afsk/afsk.h>
#include <services/modulations/cw/cw.h>
#include <services/morse/morse_afsk.h>
#include <radio_config_channels.h>

/*
 * FOX CONFIGURATION SECTION BEGIN
 */
// Defines the additional SNR level (added to the current average noise level) to wake the fox up
#define RSSI_ADDITIONAL_TRESHOLD_SNR 12

// Basic communication channels are defined in radio_config_channels.h
#define COMMUNICATION_CHANNEL CHANNEL_FOX_0

// Basic transmission power; allows to locate the fox from far distance
#define TRANSMISSION_POWER SI4438_17DBM_TX_POWER

// Nearby transmission power; helps for fine fox locating when the receiver is nearby the fox
#define TRANSMISSION_NEARBY_POWER SI4438_NEG21DBM_TX_POWER

// Uncomment below line to have more debugs around RSSI calculations
#define DEBUG_RSSI

char CALL_SIGN[] = "... .--. ...-- .. --.. -.";
char QRT[] = "--.- .-. -";
/*
 * FOX CONFIGURATION SECTION END
*/


#define FOX_STATE_RX 0
#define FOX_STATE_TX 1
#define FOX_STATE_RSSI 2

uint8_t foxState;
uint16_t rssiTreshold;

typedef struct
{
    uint8_t rssi;
    uint8_t deviation;
} average_rssi;

uint8_t sqrt(uint16_t value);
void update_rssi_treshold(average_rssi* averageRssi);
void get_average_rssi(uint8_t span_millis, uint8_t samples_count, average_rssi* result);
void stm8s_sleep(uint8_t tbr, uint8_t apr);
#define STM8_S_SLEEP_250_MILLISEC() stm8s_sleep(10, 62)
#define STM8_S_SLEEP_500_MILLISEC() stm8s_sleep(11, 62)
#define STM8_S_SLEEP_2_25_SEC() stm8s_sleep(14, 28)
#define STM8_S_SLEEP_5_SEC() stm8s_sleep(14, 62)
#define STM8_S_SLEEP_20_SEC() stm8s_sleep(15, 41)

void setup()
{
    delay(3000);

    foxState = FOX_STATE_RSSI;
    rssiTreshold = 0x7f;

    Serial_begin(115200);
    Serial_println_s("Serial init.");
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
}

void loop()
{
    if(foxState == FOX_STATE_RSSI)
    {
        // 1. go to RX state
        cw_init_rx();
        cw_start_rx(COMMUNICATION_CHANNEL);

        // 2. meassure average RSSI
        average_rssi averageRssi;
        get_average_rssi(1, 32, &averageRssi);
        // 3. display average RSSI
        Serial_print_s("RSSI average is ");
        Serial_println_i(averageRssi.rssi);

        // 4. calculate RSSI treshold
        update_rssi_treshold(&averageRssi);

        Serial_print_s("RSSI treshold is ");
        Serial_println_i(rssiTreshold);

        foxState = FOX_STATE_RX;
        return;
    }

    if(foxState == FOX_STATE_RX)
    {
        // 1. go to RX state
        cw_init_rx();
        cw_start_rx(COMMUNICATION_CHANNEL);

        // 2. check for carrier presence (PTT pressed)
        // Wait additional small delay so give the radio chip a bit of time
        // to correctly settle up after exiting the sleep mode. Benefit: average RSSI 
        // and its deviation is better calculated (it can be seen in debug logs of RSSI calculation);
        // especially deviation goes drastically low (i.e. from value of 11 to 4) 
        delay(5);
        average_rssi averageRssi;
        get_average_rssi(1, 32, &averageRssi);
        Serial_print_s("RX avgRSSI = ");
        Serial_print_i(averageRssi.rssi);
        Serial_print_s("  current trshRSSI = ");
        Serial_println_i(rssiTreshold);

        if(averageRssi.rssi >= rssiTreshold)
        {
            // Before going into transmition state wait until carrier presence
            // is gone (PTT released) - but wait no longer than 10 sec
            unsigned long start = millis();
            do
            {
                delay(500);
                get_average_rssi(1, 32, &averageRssi);
            } while ((millis() - start < 10000) && (averageRssi.rssi >= rssiTreshold));
            
            foxState = FOX_STATE_TX;
            return;
        }
        else
        {
            // update treshold
            update_rssi_treshold(&averageRssi);
            Serial_print_s("RX New trshRSSI = ");
            Serial_println_i(rssiTreshold);
        }
        
        // 3. sleep for 5s
        delay(2); // additional delay so the debug data could be sent correctly through serial port
        si4438_enter_sleep_state();
        STM8_S_SLEEP_5_SEC();

        return;
    }

    if(foxState == FOX_STATE_TX)
    {
        // 1. go to TX state
        fsk_init_tx_direct_sync_2gfsk();

        // 2. nine transmition cycles (around 9 * 65s = 585s = 9m45s in total): standard beeps
        for(uint8_t w = 0 ; w < 9 ; w ++)
        {
            fsk_start_tx(COMMUNICATION_CHANNEL);
            si4438_set_tx_power(TRANSMISSION_POWER);
            delay(500); // so the squelch on receiver could be opened

            // at first send call sign (around 5 seconds)
            morse_afsk_send_word(CALL_SIGN);
            delay(500);

            // standard beeps in the cycle (around 20 seconds)
            for(uint8_t q = 0 ; q < 8 ; q ++)
            {
                Serial_print_s("TX beep no. ");
                Serial_println_i(q);

                afsk_tone(700, 500000ul);
                delay(250);
                afsk_tone(800, 500000ul);
                delay(250);
                afsk_tone(900, 500000ul);
                delay(500);
            }

            // transmit with low power (around 20 seconds)
            si4438_set_tx_power(TRANSMISSION_NEARBY_POWER);
            afsk_tone(800, 20000000ul);

            fsk_stop_tx();

            // sleep for 20 seconds
            si4438_enter_sleep_state();
            STM8_S_SLEEP_20_SEC();
        }

        // 3. last transmition (around 30 seconds in total): fast beeps
        fsk_start_tx(COMMUNICATION_CHANNEL);
        si4438_set_tx_power(TRANSMISSION_POWER);
        delay(500); // so the squelch on receiver could be opened
        // at first send call sign
        morse_afsk_send_word(CALL_SIGN);
        delay(500);
        // beep faster to signalise it will go to sleep
        for(uint8_t q = 0 ; q < 8 ; q ++)
        {
            Serial_print_s("TX beep no. ");
            Serial_println_i(q);

            afsk_tone(900, 500000ul);
            delay(250);
            afsk_tone(900, 500000ul);
            delay(250);
            afsk_tone(900, 500000ul);
            delay(250);
            afsk_tone(900, 500000ul);
            delay(250);
        }

        // send QRT signal
        delay(500);
        morse_afsk_send_word(QRT);
        delay(500);

        fsk_stop_tx();

        // don't sleep after the last transmition cycle 

        // 4. go to FOX_STATE_RSSI state
        foxState = FOX_STATE_RSSI;

        return;
    }
}

void update_rssi_treshold(average_rssi* averageRssi)
{
    rssiTreshold = (uint16_t)averageRssi->rssi + (uint16_t)averageRssi->deviation + (uint16_t)RSSI_ADDITIONAL_TRESHOLD_SNR;

    if(rssiTreshold > 127)
    {
        rssiTreshold = 127;
    }
}

void get_average_rssi(uint8_t span_millis, uint8_t samples_count, average_rssi* result)
{
    #ifdef DEBUG_RSSI
    Serial_println_s("D get_average_rssi begin");
    #endif

    uint32_t rssiSumm = 0;
    uint32_t rssiSqSumm = 0;
    for(uint8_t q = 0 ; q < samples_count; q++)
    {
        uint8_t rssi;
        si4438_get_rssi(&rssi);

        rssiSumm += rssi;
        rssiSqSumm += ((uint16_t)rssi) * ((uint16_t)rssi);

        #ifdef DEBUG_RSSI
        Serial_print_s("D RSSI is ");
        Serial_println_i(rssi);
        #endif

        delay(span_millis);
    }
    uint8_t averageRssi = rssiSumm / samples_count;
    // https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Na%C3%AFve_algorithm
    uint16_t variance = (rssiSqSumm - rssiSumm * rssiSumm / samples_count ) / (samples_count - 1);
    uint8_t deviation = sqrt(variance);

    #ifdef DEBUG_RSSI
    Serial_print_s("D avgRSSI= ");
    Serial_print_i(averageRssi);
    Serial_print_s(" variance= ");
    Serial_print_i(variance);
    Serial_print_s(" deviation= ");
    Serial_println_i(deviation);
    #endif

    #ifdef DEBUG_RSSI
    Serial_println_s("D get_average_rssi end");
    #endif

    result->rssi = averageRssi;
    result->deviation = deviation;
}

uint8_t sqrt(uint16_t value)
{
    for(uint16_t q = 0; q < 20 ; q ++)
    {
        uint16_t square = q * q;
        if(square >= value)
        {
            return q;
        }
    }
    
    return 20;
}

void stm8s_sleep(uint8_t tbr, uint8_t apr)
{
    // How to calculate the register values:
    // RM0016_STM8S_and_STM8AF.pdf page 116 Table 25

    // Set the TimeBase
    AWU->TBR &= (uint8_t)(~AWU_TBR_AWUTB);
    AWU->TBR |= tbr;
    // Set the APR divider
    AWU->APR &= (uint8_t)(~AWU_APR_APR);
    AWU->APR |= apr;

    // Enable AWU peripheral
    AWU->CSR |= AWU_CSR_AWUEN;

    //... and enter halt mode. AWU will wake it up after specific amount of time.
    halt();

    // Disable AWU peripheral
    AWU->CSR &= (uint8_t)(~AWU_CSR_AWUEN);
    // No AWU timebase
    AWU->TBR = (uint8_t)(~AWU_TBR_AWUTB);
}

/**
  * @brief Auto Wake Up Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(AWU_IRQHandler, 1)
{
    AWU->CSR &= (uint8_t)(~AWU_CSR_AWUF);
}
