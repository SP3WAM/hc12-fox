#include <stdint.h>
#include <stdbool.h>


#define INCOMING_BUFFER_MAX_WORDS_IN_LINE 6
#define INCOMING_BUFFER_MAX_CHARACTERS_IN_WORD 12

typedef struct command_params
{
    char readBuffer[INCOMING_BUFFER_MAX_WORDS_IN_LINE][INCOMING_BUFFER_MAX_CHARACTERS_IN_WORD + 1];
	uint8_t readRowIndex;
	uint8_t readColumnIndex;
	bool endOfWordDetected;
	bool commandDetected;
} CommandParams;

void command_params_reset(CommandParams* commandParams);

uint8_t command_params_get_number_of_parameters(CommandParams* commandParams);

char* command_params_get_param(CommandParams* commandParams, uint8_t index);

bool command_params_is_command_detected(CommandParams* commandParams);

bool command_params_append_char(CommandParams* commandParams, char data);
