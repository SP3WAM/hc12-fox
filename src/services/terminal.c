#include <Arduino.h>
#include "terminal.h"

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

	char* command = getParam(&terminal_command_params, 0);

	if(strcmp(command, "") == 0)
	{
		reset(&terminal_command_params);
		terminal_print_terminal_ready();

		return;
	}

	Serial_print_s(command);
	Serial_println_s(": unknown command");

	reset(&terminal_command_params);
	terminal_print_terminal_ready();
}

bool terminal_read_command(CommandParams* commandParams)
{
	while(Serial_available() > 0)
	{
		char byte = Serial_read();
		// echo the character back to the terminal
		Serial_write(byte);
		
		if(!appendChar(commandParams, byte))
		{
			// some kind of error while apending char
			Serial_flush();
			Serial_println();
			Serial_println_s("error: terminal buffer overflow");
			Serial_flush();

            return false;
		}
		
	}

	return isCommandDetected(commandParams);
}

void terminal_print_terminal_ready()
{
	Serial_print_s("fox$");
}