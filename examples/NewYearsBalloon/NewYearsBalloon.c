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
#define COMMUNICATION_CHANNEL CHANNEL_433_500

// Basic power levels are defined in si4438.h
#define TRANSMISSION_POWER SI4438_16DBM_TX_POWER

bool chipConnected = false;

// change this to make the song slower or faster
int tempo = 120;

// duration of a whole note in ms
int wholenote = 0;
int divider = 0;
int noteDuration = 0;

void stm8s_sleep(uint8_t tbr, uint8_t apr);
#define STM8_S_SLEEP_20_SEC() stm8s_sleep(15, 41)
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
    //Serial_println_s(" OK");

    // sending startup config is mandatory (especially POWER_UP which should 
    // be the first command of this config) 
    Serial_print_s("Si4438 apply startup config... ");
    if(si4438_apply_startup_config() == false)
    {
        Serial_println_s(" failed");
    }
    //Serial_println_s(" OK");

    // configure the TX power
    Serial_print_s("Si4438 setting TX power...");
    if(si4438_set_tx_power(TRANSMISSION_POWER) == false)
    {
        Serial_println_s(" failed");
    }
    //Serial_println_s(" OK");

    // Init fake F3E transmission mode
    Serial_print_s("Si4438 setting fake F3E mode...");
    if(fsk_init_tx_direct_sync_2gfsk() == false)
    {
        Serial_println_s(" failed");
    }
    //Serial_println_s(" OK");
}

void loop()
{
    // Start the Tx mode, it will generate the carrier during the whole melody time,
    // which will make nice sound hearable in the receiver (no squelch involved during
    // whole time) 
    fsk_start_tx(COMMUNICATION_CHANNEL);
    delay(500); // a small delay to let the squelch open RX
    int weWishYouNotesCount = sizeof(we_wish_you_1) / sizeof(we_wish_you_1[0]);
    play_melody(we_wish_you_1, weWishYouNotesCount);
    weWishYouNotesCount = sizeof(we_wish_you_2) / sizeof(we_wish_you_2[0]);
    play_melody(we_wish_you_2, weWishYouNotesCount);
    delay(500);
    si4438_enter_sleep_state();
    STM8_S_SLEEP_20_SEC();
    STM8_S_SLEEP_20_SEC();
    STM8_S_SLEEP_20_SEC();

    fsk_start_tx(COMMUNICATION_CHANNEL);
    delay(500); // a small delay to let the squelch open RX
    // int neverGonnaGiveYouUpNotesCount = sizeof(never_gonna_give_you_up_1) / sizeof(never_gonna_give_you_up_1[0]);
    // play_melody(never_gonna_give_you_up_1, neverGonnaGiveYouUpNotesCount);
    int neverGonnaGiveYouUpNotesCount = sizeof(never_gonna_give_you_up_2) / sizeof(never_gonna_give_you_up_2[0]);
    play_melody(never_gonna_give_you_up_2, neverGonnaGiveYouUpNotesCount);
    neverGonnaGiveYouUpNotesCount = sizeof(never_gonna_give_you_up_3) / sizeof(never_gonna_give_you_up_3[0]);
    play_melody(never_gonna_give_you_up_3, neverGonnaGiveYouUpNotesCount);
    delay(500);
    si4438_enter_sleep_state();
    STM8_S_SLEEP_20_SEC();
    STM8_S_SLEEP_20_SEC();
    STM8_S_SLEEP_20_SEC();

    fsk_start_tx(COMMUNICATION_CHANNEL);
    delay(500); // a small delay to let the squelch open RX
    int lastChristmasNotesCount = sizeof(last_christmas) / sizeof(last_christmas[0]);
    play_melody(last_christmas, lastChristmasNotesCount);
    delay(500);
    si4438_enter_sleep_state();
    STM8_S_SLEEP_20_SEC();
    STM8_S_SLEEP_20_SEC();
    STM8_S_SLEEP_20_SEC();

    fsk_start_tx(COMMUNICATION_CHANNEL);
    delay(500); // a small delay to let the squelch open RX
    int takeOnMeNotesCount = sizeof(take_on_me) / sizeof(take_on_me[0]);
    play_melody(take_on_me, takeOnMeNotesCount);
    play_melody(take_on_me, takeOnMeNotesCount);
    play_melody(take_on_me, takeOnMeNotesCount);
    delay(500);
    si4438_enter_sleep_state();
    STM8_S_SLEEP_20_SEC();
    STM8_S_SLEEP_20_SEC();
    STM8_S_SLEEP_20_SEC();

    fsk_start_tx(COMMUNICATION_CHANNEL);
    delay(500); // a small delay to let the squelch open RX
    int finalCountdownNotesCount = sizeof(final_countdown_1) / sizeof(final_countdown_1[0]);
    play_melody(final_countdown_1, finalCountdownNotesCount);
    play_melody(final_countdown_1, finalCountdownNotesCount);
    finalCountdownNotesCount = sizeof(final_countdown_2) / sizeof(final_countdown_2[0]);
    play_melody(final_countdown_2, finalCountdownNotesCount);
    delay(500);
    si4438_enter_sleep_state();
    STM8_S_SLEEP_20_SEC();
    STM8_S_SLEEP_20_SEC();
    STM8_S_SLEEP_20_SEC();
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

void stm8s_sleep(uint8_t tbr, uint8_t apr)
{
    // How to calculate the register values:
    // RM0016_STM8S_and_STM8AF.pdf page 116 Table 25

    // Set the TimeBase
    AWU->TBR &= (uint8_t)(~AWU_TBR_AWUTB);
    AWU->TBR |= tbr;
    // Set the APR divider
    AWU->APR &= (uint8_t)(~AWU_APR_APR);
    AWU->APR |= apr;

    // Enable AWU peripheral
    AWU->CSR |= AWU_CSR_AWUEN;

    //... and enter halt mode. AWU will wake it up after specific amount of time.
    halt();

    // Disable AWU peripheral
    AWU->CSR &= (uint8_t)(~AWU_CSR_AWUEN);
    // No AWU timebase
    AWU->TBR = (uint8_t)(~AWU_TBR_AWUTB);
}

/**
  * @brief Auto Wake Up Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(AWU_IRQHandler, 1)
{
    AWU->CSR &= (uint8_t)(~AWU_CSR_AWUF);
}