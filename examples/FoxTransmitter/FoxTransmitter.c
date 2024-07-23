#include <Arduino.h>
#include "drivers/si4438.h"
#include "services/fake_f3e.h"
#include "services/cw.h"

bool chipConnected = false;
unsigned long lastTxStartMillis = 0;
bool isTx = false;

#define DOT_DURATION_MILLIS  150
#define DASH_DURATION_MILLIS 3 * DOT_DURATION_MILLIS

void stm8s_sleep(uint8_t tbr, uint8_t apr);
#define STM8_S_SLEEP_5_SEC() stm8s_sleep(14, 62)
#define STM8_S_SLEEP_250_MILLISEC() stm8s_sleep(10, 62)
#define STM8_S_SLEEP_500_MILLISEC() stm8s_sleep(11, 62)
#define STM8_S_SLEEP_20_SEC() stm8s_sleep(15, 41)

void sendDot();
void sendDash();
void sendMOE();
void sendMOI();
void sendMOS();
void sendMOH();
void sendMO5();

void rx_pin_changed(void);
bool volatile pinChanged = false;

void setup()
{
    delay(3000);

    Serial_begin(115200);
    Serial_println_s("Serial init.");
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
    if(si4438_set_tx_power(SI4438_17DBM_TX_POWER) == false)
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

    // start RX
    Serial_print_s("Si4438 setting fake CW RX mode...");
    if(cw_init_rx() == false)
    {
        Serial_println_s(" failed");
    }
    else
    {
        Serial_println_s(" OK");
    }
    Serial_print_s("Si4438 start listening...");
    if(cw_start_rx(0) == false)
    {
        Serial_println_s(" failed");
    }
    else
    {
        Serial_println_s(" OK");
    }

    // attach interrupt to PB4 as INPUT pull up
    // must be called after cw_init_rx method (where PB4 is configured as input pullup)
    GPIO_Init(GPIOB, GPIO_PIN_4, GPIO_MODE_IN_PU_IT);
    disableInterrupts();
    EXTI_SetExtIntSensitivity( EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_FALL);  
    enableInterrupts();
    attachInterrupt(INT_PORTB & 0xFF, rx_pin_changed, 0);
}

void loop()
{
    if(pinChanged)
    {
        pinChanged = false;
        Serial_println_s("RX changed");
    }
}

void rx_pin_changed(void) 
{
  pinChanged = 1;
}

void sendMOE()
{
    Serial_println_s("sending MOE...");

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
    Serial_println_s("sending MOI...");

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
    Serial_println_s("sending MOS...");

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
    Serial_println_s("sending MOH...");

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
    Serial_println_s("sending MO5...");

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
    fake_f3e_tone(700, ((unsigned long)DOT_DURATION_MILLIS) * ((unsigned long)1000));
}

void inline sendDash()
{
    fake_f3e_tone(700, ((unsigned long)DASH_DURATION_MILLIS) * ((unsigned long)1000));
}

void stm8s_sleep(uint8_t tbr, uint8_t apr)
{
    Serial_println_s("stm8s_sleep_5sec() begin");
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
