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

bool si4438_get_property(uint16_t prop, uint8_t* value)
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
    if(doAPI(cmd, sizeof(cmd), NULL, 0) == false)
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
    if(doAPI(cmd, sizeof(cmd), NULL, 0) == false)
    {
        return false;
    }

    return true;
}

// https://github.com/IU5HKU/Si4463-CW-Beacon/blob/master/src/si4463/si4463.cpp
// activateTX()
// 0x31, channel, 0, 0, 0, 0, 0
bool si4438_enter_tx_state()
{
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

bool si4438_get_part_info(uint8_t* part_info)
{
    uint8_t cmd[1]= { SI4438_CMD_PART_INFO };
    if(doAPI(cmd, sizeof(cmd), part_info, 8) == false)
    {
        return false;
    }
}

bool si4438_get_func_info(uint8_t* func_info)
{
    uint8_t cmd[1]= { SI4438_CMD_FUNC_INFO };
    if(doAPI(cmd, sizeof(cmd), func_info, 6) == false)
    {
        return false;
    }
}

// TODO: remove this functionality soon as it consumes too much of FLASH memory (binary doesn't fit into flash)
// bool si4438_set_base_frequency(unsigned long freq_Hz)
// {
//     // freq[Hz] = (fc_inte + fc_frac / 2^19) * Npresc * freq_xo / outdiv
//     double Npresc = 2.0;
//     double freq_xo = 30000000.0;
//     double outdiv = 8.0;
//     double freq = freq_Hz;

//     // To calculate fc_inte we assume fc_frac is zero and we substract 1 to fulfil the condition from the datasheet, thus:
//     // fcinte = freq * outdiv / Npresc / freq_xo - 1;
//     double fc_inte_double = freq_Hz / 7500000.0;
//     uint8_t fc_inte = ((uint8_t)fc_inte_double) - 1;

//     // Having a base value of fc_inte we calculate fc_frac from the equation
//     // fc_frac = (freq * outdiv / 2 / freq_xo - fc_inte) * (2^19)
//     double fc_frac_double = (freq - fc_inte * 7500000.0) * 0.069905;
//     uint32_t fc_frac = (uint32_t)fc_frac_double;

//     Serial_print_s("fc_inte = ");
//     Serial_println_u(fc_inte);
//     Serial_print_s("fc_frac = ");
//     Serial_println_u(fc_frac);

//     return true;
// }