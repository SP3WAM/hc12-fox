#include <SPI.h>
#include <stdint.h>
#include "si4438.h"

#define NEW_HC_12 //new version of HC-12 has different pin setting

#ifdef NEW_HC_12
  #define SI4438_nSEL PD3
#else
  #define SI4438_nSEL PD2
#endif
#define SI4438_SDN  PD4

#define SI4438_CMD_PART_INFO 0x01
#define SI4438_CMD_POWER_UP 0x02	
#define SI4438_CMD_SET_PROPERTY 0x11
#define SI4438_CMD_GET_PROPERTY 0x12
#define SI4438_CMD_READ_CMD_BUFF 0x44

#define SI4438_PROPERTY_PA_PWR_LVL 0x2201

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

bool si4438_send_command(uint8_t command, uint8_t* args, uint8_t len)
{
    // wait until chip accepts commands
    if(si4438_wait_for_response(NULL, 0) == false)
    {
        return false;
    }

    si4438_cs_low();
    SPI_transfer(command);

    for(uint8_t q = 0 ; q < len ; q ++)
    {
        SPI_transfer(args[q]);
    }

    SPI_transfer(0xFF);
    si4438_cs_high();

    return true;
}

bool setProperty(uint16_t prop, uint8_t value)
{
    uint8_t args[4];
    args[0] = prop >> 8; // property group
    args[1] = 1; // just one property
    args[2] = prop & 0x00ff; // property number
    args[3] = value;  // property value

    return si4438_send_command(SI4438_CMD_SET_PROPERTY, args, 4);
}

bool getProperty(uint16_t prop, uint8_t* value)
{
    uint8_t args[3];
    args[0] = prop >> 8; // property group
    args[1] = 1; // just one property
    args[2] = prop & 0x00ff; // property number

    if(si4438_send_command(SI4438_CMD_GET_PROPERTY, args, 3) == false)
    {
        return false;
    }

    if(si4438_wait_for_response(value, 1) == false)
    {
        return false;
    }

    return true;
}

bool si4438_is_chip_connected()
{
    // send the command
    if(si4438_send_command(SI4438_CMD_PART_INFO, NULL, 0) == false)
    {
        return false;
    }

    // get the response
    uint8_t buff[8];
    if(si4438_wait_for_response(buff, 8) == false)
    {
        return false;
    }

    //for(uint8_t q = 0 ; q < 8 ; q++)
    //{
    //    Serial_print_i(buff[q]);
    //    Serial_print_s(" ");
    //}
    //Serial_println_s("");

    if(buff[1] == 0x44 && buff[2] == 0x38)
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

bool si4438_power_up()
{
    // POWER_UP documentation:
    // PACH = 0 meaning Copy selected functional image from OTP and boot device
    // FUNC = 1 meaning Boot main application image
    // XO_FREQ = 0x01C9C380 (default 30MHz) Frequency of TCXO or external crystal oscillator in Hz.
    uint8_t args[] = {0x01, 0x00, 0x01, 0xC9, 0xC3, 0x80};

    return si4438_send_command(SI4438_CMD_POWER_UP, args, 6);
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

