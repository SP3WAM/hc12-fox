#include <Arduino.h>
#include "drivers/si4438.h"
#include "services/fake_f3e.h"
#include "services/cw.h"

#define FOX_STATE_RX 0
#define FOX_STATE_TX 1
#define FOX_STATE_RSSI 2

uint8_t foxState;
uint8_t rssiTreshold;

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

    // The thershold avlue below is used to configure detection of carrier in Rx mode
    // Tests shows lower values of the threshold give a false carrier detection (threshold is below noise level?)
    // Need to choose the threshold experimental but it may vary depending on the environment (urban or non-urban).
    // It is important to choose the lowest possible value of the threshold.  
    // 0x00=  0 -> -132dBm carrier detected all the time (threshold value below noise level?)
    // 0x3F= 63 -> -100dBm carrier detected all the time
    // 0x4E= 78 ->  -93dBm carrier detected all the time
    // 0x5E= 94 -> -85dBm it works, carrier detected only when PTT of handy radio (2.5W) is pressed
    // 0x7F=127 -> -68dBm it works, carrier detected only when PTT of handy radio (2.5W) is pressed
    // 0xbF=191 -> -36dBm it works, carrier detected only when PTT of handy radio (2.5W) is pressed
    // 0xFF=255 ->  -4dBm carrier not detected at all (even if handy radio transmits with 5W). I don't know why.
    Serial_print_s("Si4438 setting modem RSSI treshold...");
    if(si4438_setProperty(SI4438_PROPERTY_MODEM_RSSI_THRESH, 0x5e) == false)
    {
        Serial_println_s(" failed");
    }
    else
    {
        Serial_println_s(" OK");
    }

    cw_init_rx();
    cw_start_rx(0);
}

void loop()
{
    if(foxState == FOX_STATE_RSSI)
    {
        // 1. go to RX state
        cw_init_rx();
        cw_start_rx(0);

        // 2. meassure average RSSI
        uint16_t averageRssi = get_average_rssi(50, 32);
        // 3. display average RSSI
        Serial_print_s("RSSI average is ");
        Serial_println_i(averageRssi);

        // 4. calculate RSSI treshold
        rssiTreshold = (uint8_t)((averageRssi + 6) % 256);
        Serial_print_s("RSSI treshold is ");
        Serial_println_i(rssiTreshold);

        // 5. set new treshold
        si4438_setProperty(SI4438_PROPERTY_MODEM_RSSI_THRESH, rssiTreshold);

        foxState = FOX_STATE_RX;
        return;
    }

    if(foxState == FOX_STATE_RX)
    {
        Serial_print_s("Fox in RX state.");

        // 1. go to RX state
        cw_init_rx();
        cw_start_rx(0);

        // 2. for 160ms check for carrier presence
        uint8_t averageRssi = get_average_rssi(5, 32);
        Serial_print_s("RSSI average is ");
        Serial_println_i(averageRssi);
        if(averageRssi >= rssiTreshold)
        {
            foxState = FOX_STATE_TX;
            return;
        }
        
        // 3. sleep for 2.25s
        si4438_enter_sleep_state();
        STM8_S_SLEEP_2_25_SEC();

        return;
    }

    if(foxState == FOX_STATE_TX)
    {
        Serial_print_s("Fox in TX state.");

        // 1. go to TX state
        fake_f3e_init_tx_direct_sync_2gfsk();

        // 2. transmit beeps for 30 minutes
        // for(uint8_t w = 0 ; w < 30 ; w ++)
        // {
        //     // the loop below takes 20 seconds
        //     for(uint8_t q = 0 ; q < 20 ; q ++)
        //     {
        //         // 20 * (500ms + 500ms) = 20 * 1s = 20s
        //         fake_f3e_start_tx(0); // channel 0: 434.100 MHz
        //         fake_f3e_tone(700, 500000ul);
        //         fake_f3e_stop_tx();
        //         si4438_enter_sleep_state();
        //         STM8_S_SLEEP_250_MILLISEC();
        //         STM8_S_SLEEP_500_MILLISEC();
        //     }

        //     // sleep for 40 seconds
        //     STM8_S_SLEEP_20_SEC();
        //     STM8_S_SLEEP_20_SEC();
        // }

        // 2. transmit beeps for 20 seconds
        for(uint8_t q = 0 ; q < 20 ; q ++)
        {
            // 20 * (500ms + 500ms) = 20 * 1s = 20s
            fake_f3e_start_tx(0); // channel 0: 434.100 MHz
            fake_f3e_tone(700, 500000ul);
            fake_f3e_stop_tx();
            si4438_enter_sleep_state();
            STM8_S_SLEEP_250_MILLISEC();
            STM8_S_SLEEP_500_MILLISEC();
        }

        // 3. go to FOX_STATE_RSSI state
        foxState = FOX_STATE_RSSI;

        return;
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

        Serial_print_s("RSSI is ");
        Serial_println_i(rssi);

        delay(span_millis);
    }
    uint8_t averageRssi = rssiSumm / samples_count;

    return averageRssi;
}

void stm8s_sleep(uint8_t tbr, uint8_t apr)
{
    Serial_println_s("stm8s_sleep() begin");
    // How to calculate the register values:
    // RM0016_STM8S_and_STM8AF.pdf page 116 Table 25

    // Set the TimeBase 2.080 s - 5.120 s
    AWU->TBR &= (uint8_t)(~AWU_TBR_AWUTB);
    AWU->TBR |= tbr;
    // Set the APR divider APR = 5.0 sec / 5 / 2^11 * 128000 = 62.5
    AWU->APR &= (uint8_t)(~AWU_APR_APR);
    AWU->APR |= apr;

    // Enable AWU peripheral
    AWU->CSR |= AWU_CSR_AWUEN;

    //... and enter halt mode. AWU will wake it up after specific amount of time.

    Serial_println_s("about to call halt()...");
    delay(100); // this is required to send data to serial port before halt is executed
    halt();
    Serial_println_s("... exiting from halt()");

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
