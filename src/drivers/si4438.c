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
#define SI4438_CMD_READ_CMD_BUFF 0x44

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

bool si4438_is_chip_connected()
{
    if(si4438_wait_for_response(NULL, 0) == false)
    {
        return false;
    }

    // send the command
    si4438_cs_low();
    SPI_transfer(SI4438_CMD_PART_INFO);
    si4438_cs_high();

    // get the response
    uint8_t buff[8];
    if(si4438_wait_for_response(buff, 8) == false)
    {
        return false;
    }

    for(uint8_t q = 0 ; q < 8 ; q++)
    {
        Serial_print_i(buff[q]);
        Serial_print_s(" ");
    }
    Serial_println_s("");

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
