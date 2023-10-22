#include <string.h>
#include <SPI.h>
#include <stdint.h>
#include "si4438.h"
#include "../radio_config_Si4438.h"

#define NEW_HC_12 //new version of HC-12 has different pin setting

#ifdef NEW_HC_12
  #define SI4438_nSEL PD3
#else
  #define SI4438_nSEL PD2
#endif
#define SI4438_SDN  PD4
#define SI4438_DIRECT_MODE_TX_PIN PB4 // connected to GPIO0 of Si4438

static const uint8_t STARTUP_CONFIG[] PROGMEM = RADIO_CONFIGURATION_DATA_ARRAY;

void si4438_cs_low()
{
    digitalWrite(SI4438_nSEL, LOW);
}

void si4438_cs_high()
{
    digitalWrite(SI4438_nSEL, HIGH);
}

uint8_t si4438_get_response(void* buff, uint8_t len)
{
    uint8_t cts = 0;

    si4438_cs_low();
    SPI_transfer(SI4438_CMD_READ_CMD_BUFF);
    
    // get CTS value
    cts = SPI_transfer(0xFF);
    if(cts == 0xFF)
    {
        // get response data
        for(uint8_t i=0 ; i < len ; i++)
        {
            ((uint8_t*)buff)[i] = SPI_transfer(0xFF);
        }   
    }
    
    si4438_cs_high();

    return cts;
}

bool si4438_wait_for_response(void* out, uint8_t outLen)
{
    long beginMillis = millis();

	while(si4438_get_response(out, outLen) != 0xFF)
	{
        delayMicroseconds(10);
		if(millis() > beginMillis + 500)
		{
            // timeout
			return false;
		}
	}

	return true;
}

bool doAPI(uint8_t* data, uint8_t len, uint8_t* out, uint8_t outLen)
{
    // wait until chip accepts commands
    if(si4438_wait_for_response(NULL, 0) == false)
    {
        return false;
    }

    si4438_cs_low();
    for(uint8_t i = 0 ; i < len ; i++)
    {
        SPI_transfer(data[i]);
    }
    si4438_cs_high();

    if(out != NULL)
    {
        return si4438_wait_for_response(out, outLen);
    }				

    return true;
}

bool setProperty(uint16_t prop, uint8_t value)
{
    uint8_t cmd[5];
    cmd[0] = SI4438_CMD_SET_PROPERTY;
    cmd[1] = prop >> 8; // property group
    cmd[2] = 1; // just one property
    cmd[3] = prop & 0x00ff; // property number
    cmd[4] = value;  // property value

    return doAPI(cmd, sizeof(cmd), NULL, 0);
}

bool getProperty(uint16_t prop, uint8_t* value)
{
    uint8_t cmd[4];
    cmd[0] = SI4438_CMD_GET_PROPERTY;
    cmd[1] = prop >> 8; // property group
    cmd[2] = 1; // just one property
    cmd[3] = prop & 0x00ff; // property number

    return doAPI(cmd, sizeof(cmd), value, 1);
}

bool si4438_is_chip_connected()
{
    uint8_t cmd[1]= { SI4438_CMD_PART_INFO };
    uint8_t resp[8];
    if(doAPI(cmd, sizeof(cmd), resp, sizeof(resp)) == false)
    {
        return false;
    }

    //for(uint8_t q = 0 ; q < 8 ; q++)
    //{
    //    Serial_print_i(buff[q]);
    //    Serial_print_s(" ");
    //}
    //Serial_println_s("");

    if(resp[1] == 0x44 && resp[2] == 0x38)
    {
        return true;
    }

    return false;
}

bool si4438_init_hw()
{   
    pinMode(SI4438_nSEL, OUTPUT);
    si4438_cs_high();

    SPI_begin();
 
    pinMode(SI4438_SDN, OUTPUT);
    digitalWrite(SI4438_SDN, HIGH);
    delay(50);
    digitalWrite(SI4438_SDN, LOW);
    delay(50);
    
    return true;
}

bool si4438_apply_startup_config()
{
    bool result = true;
    uint8_t buff[17];
	for(uint16_t i = 0 ; i < sizeof(STARTUP_CONFIG) ; i++)
	{
		memcpy(buff, &STARTUP_CONFIG[i], sizeof(buff));
		result &= doAPI(&buff[1], buff[0], NULL, 0);
		i += buff[0];
	}

    return result;
}

bool si4438_set_tx_power(uint8_t pwr)
{
    if(setProperty(SI4438_PROPERTY_PA_PWR_LVL, pwr) == false)
    {
        return false;
    }

    uint8_t readPwr;
    if(getProperty(SI4438_PROPERTY_PA_PWR_LVL, &readPwr) == false)
    {
        return false;
    }

    if(pwr == readPwr)
    {
        return true;
    }

    return false;
}

bool si4438_init_cw()
{
    // configure GPIO
    pinMode(SI4438_DIRECT_MODE_TX_PIN, OUTPUT);
    digitalWrite(SI4438_SDN, LOW);
    
    uint8_t gpioCmd[8];
    gpioCmd[0] = SI4438_CMD_GPIO_PIN_CFG;
    gpioCmd[1] = 0b01000100; // GPIO0: CMOS input, pull up enabled
    gpioCmd[2] = 0x00; // GPIO1
    gpioCmd[3] = 0b0000011; // GPIO2: CMOS output driven high, pull up disabled
    gpioCmd[4] = 0x0000010; // GPIO3: CMOS output driven low, pull up disabled
    gpioCmd[5] = 0x00; // NIRQ
    gpioCmd[6] = 0x00; // SDO
    gpioCmd[7] = 0x00; // GEN_CONFIG

    if(doAPI(gpioCmd, sizeof(gpioCmd), NULL, 0) == false)
    {
        return false;
    }

    // configure CW Tx mode
    //   TX_DIRECT_MODE_TYPE[0] = 0b1;   Direct mode operates in asynchronous mode, applies to TX only. GFSK is not supported.
    // TX_DIRECT_MODE_GPIO[1:0] = 0b00;  TX direct mode uses GPIO0 as data source.
    //          MOD_SOURCE[1:0] = 0b01;  The modulation is sourced in real-time (i.e., TX Direct Mode) from a GPIO pin, as selected by the TX_DIRECT_MODE_GPIO field.
    //            MOD_TYPE[2:0] = 0b000; CW
    uint8_t value = 0b10001000; 
    if(setProperty(SI44338_PROPERTY_MODEM_MOD_TYPE, value) == false)
    {
        return false;
    }

    // put into TX mode
    uint8_t startTxCmd[5];
    startTxCmd[0] = SI4438_CMD_START_TX;
    startTxCmd[1] = 0; // channel 0
    startTxCmd[2] = 0;
    startTxCmd[3] = 0;
    startTxCmd[4] = 0;
    if(doAPI(startTxCmd, sizeof(startTxCmd), NULL, 0) == false)
    {
        return false;
    }

    return true;
}

void si4438_cw_on()
{
    digitalWrite(SI4438_SDN, HIGH);
}

void si4438_cw_off()
{
    digitalWrite(SI4438_SDN, LOW);
}
