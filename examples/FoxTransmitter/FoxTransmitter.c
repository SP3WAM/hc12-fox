#include <Arduino.h>
#include "drivers/si4438.h"
#include "services/fake_f3e.h"
#include "services/cw.h"

/*
 * FOX CONFIGURATION SECTION BEGIN
 */
// Defines the minimal SNR level to wake the fox up
#define RSSI_TRESHOLD_SNR 20

// Communication channel:
// The step between single channels is 12.5 kHz, but foxes communicate on channels with 50 kHz step.
//  0 434.100 MHz this is the base channel of all foxes (see also radio_config_Si4438.h)
//  4 434.150 MHz
//  8 434.200 MHz
// 12 434.250 MHz
// 16 434.300 MHz
// 20 434.350 MHz
// 24 434.400 MHz
// 28 434.450 MHz
// 32 434.500 MHz
// 36 434.550 MHz this is the last possible channel according to IARU Region 1 UHF band plan
#define COMMUNICATION_CHANNEL 0
/*
 * FOX CONFIGURATION SECTION END
*/


#define FOX_STATE_RX 0
#define FOX_STATE_TX 1
#define FOX_STATE_RSSI 2

uint8_t foxState;
uint16_t rssiTreshold;

void set_rssi_treshold(uint16_t treshold);
uint8_t get_average_rssi(uint8_t span_millis, uint8_t samples_count);
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
    if(si4438_set_tx_power(SI4438_17DBM_TX_POWER) == false)
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
        uint16_t averageRssi = get_average_rssi(16, 32);
        // 3. display average RSSI
        Serial_print_s("RSSI average is ");
        Serial_println_i(averageRssi);

        // 4. calculate RSSI treshold
        set_rssi_treshold(averageRssi + RSSI_TRESHOLD_SNR);
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

        // 2. check for carrier presence
        uint8_t averageRssi = get_average_rssi(16, 32);
        Serial_print_s("RX avgRSSI = ");
        Serial_print_i(averageRssi);
        Serial_print_s("  trshRSSI = ");
        Serial_println_i(rssiTreshold);

        if(averageRssi >= rssiTreshold)
        {
            foxState = FOX_STATE_TX;
            return;
        }
        else
        {
            // update treshold
            set_rssi_treshold(averageRssi + RSSI_TRESHOLD_SNR);
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
        fake_f3e_init_tx_direct_sync_2gfsk();

        // 2. transmition cycle: 10 minutes
        for(uint8_t w = 0 ; w < 10 ; w ++)
        {
            // transmit beeps for 20 seconds
            for(uint8_t q = 0 ; q < 20 ; q ++)
            {
                // 20 * (500ms + 500ms) = 20 * 1s = 20s
                Serial_print_s("TX beep no. ");
                Serial_println_i(q);

                fake_f3e_start_tx(COMMUNICATION_CHANNEL);
                fake_f3e_tone(700, 500000ul);
                fake_f3e_stop_tx();
                si4438_enter_sleep_state();
                STM8_S_SLEEP_500_MILLISEC();
            }

            // sleep for 40 seconds
            STM8_S_SLEEP_20_SEC();
            STM8_S_SLEEP_20_SEC();
        }

        // 3. go to FOX_STATE_RSSI state
        foxState = FOX_STATE_RSSI;

        return;
    }
}

void set_rssi_treshold(uint16_t treshold)
{
    rssiTreshold = treshold;
    if(rssiTreshold > 127)
    {
        rssiTreshold = 127;
    }
}

uint8_t get_average_rssi(uint8_t span_millis, uint8_t samples_count)
{
    uint16_t rssiSumm = 0;
    for(uint8_t q = 0 ; q < samples_count; q++)
    {
        uint8_t rssi;
        si4438_get_rssi(&rssi);

        rssiSumm += rssi;

        delay(span_millis);
    }
    uint8_t averageRssi = rssiSumm / samples_count;

    return averageRssi;
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
