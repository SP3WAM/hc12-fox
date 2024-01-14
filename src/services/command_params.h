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

void reset(CommandParams* commandParams);

uint8_t getNumberOfParameters(CommandParams* commandParams);

char* getParam(CommandParams* commandParams, uint8_t index);

bool isCommandDetected(CommandParams* commandParams);

bool appendChar(CommandParams* commandParams, char data);
