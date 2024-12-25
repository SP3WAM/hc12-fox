#include <Arduino.h>
#include <drivers/si4438.h>
#include <services/modulations/fsk/fsk.h>
#include <services/modulations/afsk/afsk.h>
#include <radio_config_channels.h>
#include <services/player/notes.h>
#include <services/player/songs/we_wish_you.h>
#include <services/player/songs/last_christmas.h>
#include <services/player/songs/final_countdown.h>
#include <services/player/songs/never_gonna_give_you_up.h>
#include <services/player/songs/take_one_me.h>

// Basic communication channels are defined in radio_config_channels.h
#define COMMUNICATION_CHANNEL CHANNEL_FOX_0

// Basic power levels are defined in si4438.h
#define TRANSMISSION_POWER SI4438_MIN_TX_POWER

bool chipConnected = false;

// change this to make the song slower or faster
int tempo = 120;

// duration of a whole note in ms
int wholenote = 0;
int divider = 0;
int noteDuration = 0;

void play_melody(Note* notes, int notesCount);

void setup()
{
    // this calculates the duration of a whole note in ms
    wholenote = ((60000 * 4) / tempo);

    delay(3000);

    Serial_begin(115200);
    si4438_init_hw();
    delay(1000);

    // at first check if the hardware is connected
    Serial_print_s("Si4438 checking hardware...");
    chipConnected = si4438_is_chip_connected();
    if(chipConnected == false)
    {
        Serial_println_s(" failed");
    }
    Serial_println_s(" OK");

    // sending startup config is mandatory (especially POWER_UP which should 
    // be the first command of this config) 
    Serial_print_s("Si4438 apply startup config... ");
    if(si4438_apply_startup_config() == false)
    {
        Serial_println_s(" failed");
    }
    Serial_println_s(" OK");

    // configure the TX power
    Serial_print_s("Si4438 setting TX power...");
    if(si4438_set_tx_power(TRANSMISSION_POWER) == false)
    {
        Serial_println_s(" failed");
    }
    Serial_println_s(" OK");

    // Init fake F3E transmission mode
    Serial_print_s("Si4438 setting fake F3E mode...");
    if(fsk_init_tx_direct_sync_2gfsk() == false)
    {
        Serial_println_s(" failed");
    }
    Serial_println_s(" OK");
}

void loop()
{
    // Start the Tx mode, it will generate the carrier during the whole melody time,
    // which will make nice sound hearable in the receiver (no squelch involved during
    // whole time) 
     fsk_start_tx(COMMUNICATION_CHANNEL);
     delay(1000); // a small delay to let the squelch open RX

    // sizeof gives the number of bytes, each int value is composed of two bytes (16 bits)
    // there are two values per note (pitch and duration), so for each note there are four bytes
    int weWishYouNotesCount = sizeof(we_wish_you_1) / sizeof(we_wish_you_1[0]);
    play_melody(we_wish_you_1, weWishYouNotesCount);
    weWishYouNotesCount = sizeof(we_wish_you_2) / sizeof(we_wish_you_2[0]);
    play_melody(we_wish_you_2, weWishYouNotesCount);
    delay(1000);

    // // int neverGonnaGiveYouUpNotesCount = sizeof(never_gonna_give_you_up_1) / sizeof(never_gonna_give_you_up_1[0]) / 2;
    // // play_melody(never_gonna_give_you_up_1, neverGonnaGiveYouUpNotesCount);
    // int neverGonnaGiveYouUpNotesCount = sizeof(never_gonna_give_you_up_2) / sizeof(never_gonna_give_you_up_2[0]) / 2;
    // play_melody(never_gonna_give_you_up_2, neverGonnaGiveYouUpNotesCount);
    // neverGonnaGiveYouUpNotesCount = sizeof(never_gonna_give_you_up_3) / sizeof(never_gonna_give_you_up_3[0]) / 2;
    // play_melody(never_gonna_give_you_up_3, neverGonnaGiveYouUpNotesCount);
    // delay(1000);

    // int lastChristmasNotesCount = sizeof(last_christmas) / sizeof(last_christmas[0]) / 2;
    // play_melody(last_christmas, lastChristmasNotesCount);
    // delay(1000);

    // int finalCountdownNotesCount = sizeof(final_countdown_1) / sizeof(final_countdown_1[0]) / 2;
    // play_melody(final_countdown_1, finalCountdownNotesCount);
    // play_melody(final_countdown_1, finalCountdownNotesCount);
    // finalCountdownNotesCount = sizeof(final_countdown_2) / sizeof(final_countdown_2[0]) / 2;
    // play_melody(final_countdown_2, finalCountdownNotesCount);
    // delay(1000);
    
    // Disable Tx mode, carrier not generated anymore
    fsk_stop_tx();

    // Enter si4438 into sleep state
    si4438_enter_sleep_state();

    // Just wait for some time before pleying it again
    delay(5000);
}

void play_melody(Note* melody, int notesCount)
{
    // iterate over the notes of the melody. 
    // Remember, the array is twice the number of notes (notes + durations)
    for (int thisNote = 0; thisNote < notesCount; thisNote ++)
    {
        // calculates the duration of each note
        divider = melody[thisNote].duration;
        if (divider > 0)
        {
            // regular note, just proceed
            noteDuration = (wholenote) / divider;
        } 
        else if (divider < 0) 
        {
            // dotted notes are represented with negative durations!!
            noteDuration = (wholenote) / abs(divider);
            noteDuration *= 1.5; // increases the duration in half for dotted notes
        }

        // we only play the note for 90% of the duration, leaving 10% as a pause
        afsk_tone(melody[thisNote].note, noteDuration * 0.9 * 1000);

        // wait for the 10% of the note duration before playing the next note.
        delay(noteDuration * 0.1);
    }
}

/*
 * Empty interrupt handler.
 */
INTERRUPT_HANDLER(AWU_IRQHandler, 1)
{

}