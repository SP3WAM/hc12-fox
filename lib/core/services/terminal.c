#include <Arduino.h>
#include "terminal.h"
#include "../drivers/si4438.h"

CommandParams terminal_command_params;

void terminal_init()
{
    terminal_print_terminal_ready();
}

void terminal_loop() {
	// check for incoming commands
	if(terminal_read_command(&terminal_command_params) == false)
	{
		return;
	}

	char* command = command_params_get_param(&terminal_command_params, 0);
	uint8_t numberOfParams = command_params_get_number_of_parameters(&terminal_command_params);

	if(strcmp(command, "") == 0)
	{
		command_params_reset(&terminal_command_params);
		terminal_print_terminal_ready();

		return;
	}

	if(numberOfParams == 2 && strcmp(command, "read") == 0)
	{
		// read command
		char* propertyString = command_params_get_param(&terminal_command_params, 1);
		unsigned long property = strtoul(propertyString, NULL, 0);
		Serial_println_s(propertyString);
		Serial_println_u(property);

		char value;
		if(si4438_get_property(property, &value))
		{
			char valueAsHexString[3];
			Serial_println_u(value);
			sprintf(valueAsHexString, "%02X", value);
			Serial_print_s("0x");
			Serial_println_s(valueAsHexString);
		}
		else
		{
			Serial_println_s("Command processing error.");
		}

		command_params_reset(&terminal_command_params);
		terminal_print_terminal_ready();

		return;
	}

	Serial_print_s(command);
	Serial_println_s(": unknown command");

	command_params_reset(&terminal_command_params);
	terminal_print_terminal_ready();
}

bool terminal_read_command(CommandParams* commandParams)
{
	while(Serial_available() > 0)
	{
		char byte = Serial_read();
		// echo the character back to the terminal
		Serial_write(byte);
		
		if(!command_params_append_char(commandParams, byte))
		{
			// some kind of error while apending char
			Serial_flush();
			Serial_println();
			Serial_println_s("error: terminal buffer overflow");
			Serial_flush();

            return false;
		}
		
	}

	return command_params_is_command_detected(commandParams);
}

void terminal_print_terminal_ready()
{
	Serial_print_s("fox$");
}