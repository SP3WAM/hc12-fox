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

bool si4438_doAPI(uint8_t* data, uint8_t len, uint8_t* out, uint8_t outLen)
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

bool si4438_setProperty(uint16_t prop, uint8_t value)
{
    uint8_t cmd[5];
    cmd[0] = SI4438_CMD_SET_PROPERTY;
    cmd[1] = prop >> 8; // property group
    cmd[2] = 1; // just one property
    cmd[3] = prop & 0x00ff; // property number
    cmd[4] = value;  // property value

    return si4438_doAPI(cmd, sizeof(cmd), NULL, 0);
}

bool si4438_get_property(uint16_t prop, uint8_t* value)
{
    uint8_t cmd[4];
    cmd[0] = SI4438_CMD_GET_PROPERTY;
    cmd[1] = prop >> 8; // property group
    cmd[2] = 1; // just one property
    cmd[3] = prop & 0x00ff; // property number

    return si4438_doAPI(cmd, sizeof(cmd), value, 1);
}

bool si4438_is_chip_connected()
{
    uint8_t resp[8];
    if(si4438_get_part_info(resp) == false)
    {
        return false;
    }

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
		result &= si4438_doAPI(&buff[1], buff[0], NULL, 0);
		i += buff[0];
	}

    return result;
}

bool si4438_set_tx_power(uint8_t pwr)
{
    if(si4438_setProperty(SI4438_PROPERTY_PA_PWR_LVL, pwr) == false)
    {
        return false;
    }

    uint8_t readPwr;
    if(si4438_get_property(SI4438_PROPERTY_PA_PWR_LVL, &readPwr) == false)
    {
        return false;
    }

    if(pwr == readPwr)
    {
        return true;
    }

    return false;
}

bool si4438_enter_sleep_state()
{
    // put into SLEEP state
    uint8_t cmd[2];
    cmd[0] = SI4438_CMD_CHANGE_STATE;
    cmd[1] = 0x01; // SLEEP state.
    if(si4438_doAPI(cmd, sizeof(cmd), NULL, 0) == false)
    {
        return false;
    }

    return true;
}

bool si4438_enter_ready_state()
{
    // put into READY state
    uint8_t cmd[2];
    cmd[0] = SI4438_CMD_CHANGE_STATE;
    cmd[1] = 0x03; // READY state.
    if(si4438_doAPI(cmd, sizeof(cmd), NULL, 0) == false)
    {
        return false;
    }

    return true;
}

// https://github.com/IU5HKU/Si4463-CW-Beacon/blob/master/src/si4463/si4463.cpp
// activateTX()
// 0x31, channel, 0, 0, 0, 0, 0
// Attention: on Si4463 the command has 6 bytes of parameters
// Attention: on SI4438 the command has 5 bytes of parameters
bool si4438_enter_tx_state(uint8_t channel)
{
    // put into TX mode
    uint8_t startTxCmd[6];
    startTxCmd[0] = SI4438_CMD_START_TX;
    startTxCmd[1] = channel;
    startTxCmd[2] = 0;
    startTxCmd[3] = 0;
    startTxCmd[4] = 0;
    startTxCmd[5] = 0;
    if(si4438_doAPI(startTxCmd, sizeof(startTxCmd), NULL, 0) == false)
    {
        return false;
    }

    return true;
}

bool si4438_enter_rx_state(uint8_t channel)
{
    // put into RX state
    uint8_t startTxCmd[8];
    startTxCmd[0] = SI4438_CMD_START_RX;
    startTxCmd[1] = channel;
    startTxCmd[2] = 0;
    startTxCmd[3] = 0;
    startTxCmd[4] = 0;
    startTxCmd[5] = 0;
    startTxCmd[6] = 0;
    startTxCmd[7] = 0;
    if(si4438_doAPI(startTxCmd, sizeof(startTxCmd), NULL, 0) == false)
    {
        return false;
    }

    return true;
}

bool si4438_get_part_info(uint8_t* part_info)
{
    uint8_t cmd[1]= { SI4438_CMD_PART_INFO };
    if(si4438_doAPI(cmd, sizeof(cmd), part_info, 8) == false)
    {
        return false;
    }
}

bool si4438_get_func_info(uint8_t* func_info)
{
    uint8_t cmd[1]= { SI4438_CMD_FUNC_INFO };
    if(si4438_doAPI(cmd, sizeof(cmd), func_info, 6) == false)
    {
        return false;
    }
}

bool si4438_get_rssi(uint8_t* rssi)
{
    uint8_t command[2];
    command[0] = SI4438_CMD_GET_MODEM_STATUS;
    command[1] = 0xFF;  // don't clear interrupts

    uint8_t response[8];

    if(si4438_doAPI(command, sizeof(command), response, sizeof(response)) == false)
    {
        return false;
    }

    *rssi = response[2];

    return true;
}
