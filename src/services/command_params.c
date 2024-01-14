#include "command_params.h"

void reset(CommandParams* commandParams)
{
	commandParams->readRowIndex = 0;
	commandParams->readColumnIndex = 0;
	commandParams->endOfWordDetected = false;
	commandParams->commandDetected = false;
}

uint8_t getNumberOfParameters(CommandParams* commandParams)
{
	return commandParams->readRowIndex + 1;
}

char* getParam(CommandParams* commandParams, uint8_t index)
{
	return commandParams->readBuffer[index];
}

bool isCommandDetected(CommandParams* commandParams)
{
	return commandParams->commandDetected;
}

bool appendChar(CommandParams* commandParams, char byte)
{
	if(byte == ' ' && commandParams->readColumnIndex == 0)
	{
		// skip extra spaces before any words
		return false;
	}

	if(byte == '\r')
	{
		// carrier return, just ignore this character
		return true;
	}

	if(byte == '\n')
	{
		// line feed, treat this as an end-of-line
		commandParams->readBuffer[commandParams->readRowIndex][commandParams->readColumnIndex] = 0;
		commandParams->commandDetected = true;

		return true;
	}

	if(byte == ' ' && commandParams->readColumnIndex > 0)
	{
		// end of word in line
		commandParams->endOfWordDetected = true;
		commandParams->readBuffer[commandParams->readRowIndex][commandParams->readColumnIndex] = 0;

		return true;
	}

	if(byte != ' ' && commandParams->endOfWordDetected)
	{
		// new word detected
		commandParams->endOfWordDetected = false;

		commandParams->readRowIndex++;
		commandParams->readColumnIndex = 0;

		if(commandParams->readRowIndex >= INCOMING_BUFFER_MAX_WORDS_IN_LINE)
		{
			// incoming buffer overflow
			reset(commandParams);

			return false;
		}
	}

	// a regular character of the word
	commandParams->readBuffer[commandParams->readRowIndex][commandParams->readColumnIndex] = byte;
	commandParams->readColumnIndex ++;

	if(commandParams->readColumnIndex > INCOMING_BUFFER_MAX_CHARACTERS_IN_WORD)
	{
		// incoming buffer overflow
		reset(commandParams);

		return false;
	}

	return true;
}
/*
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
*/