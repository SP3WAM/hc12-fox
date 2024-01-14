#include <Arduino.h>
#include "commands.h"
#include "../drivers/si4438.h"

#define COMMANDS_BUFFER_SIZE 16

char commands_read_buffer[COMMANDS_BUFFER_SIZE];
uint8_t commands_read_index = 0;

void commands_loop()
{
    if(!commands_read_command())
	{
		return;
	}

	// some command received
	if(commands_read_buffer[0] == 'r' && commands_read_index == 7)
	{
		// command r0xDDDD\n
		commands_read_buffer[7] = '\0';
		unsigned long property = strtoul(commands_read_buffer + 1, NULL, 0);
		Serial_println_i(property);
		char value;
		if(si4438_get_property(property, &value))
		{
			Serial_println_u(value);
		}
		else
		{
			Serial_println_s("Command processing error.");
		}

		commands_read_index = 0;
		return;
	}

	// reset input buffer
	commands_read_index = 0;
	Serial_println_s("Reset read buffer");
}

bool commands_read_command()
{
	while(Serial_available() > 0)
	{
		char byte = Serial_read();
		commands_read_buffer[commands_read_index] = byte;
		
		// echo the character back to the terminal
		Serial_write(byte);
		
		if(byte == '\n')
		{
			// command received
			return true;
		}
		
		commands_read_index ++;
		if(commands_read_index >= COMMANDS_BUFFER_SIZE)
		{
			// buffer overflow
			Serial_flush();
			Serial_println();
			Serial_println_s("error: terminal buffer overflow");
			Serial_flush();

			commands_read_index = 0;

			return false;
		}
	}

	return false;
}