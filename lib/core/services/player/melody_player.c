#include <Arduino.h>
#include <services/modulations/afsk/afsk.h>
#include "melody_player.h"

// change this to make the song slower or faster
int tempo = 120;

// duration of a whole note in ms
int wholenote = 0;
int divider = 0;
int noteDuration = 0;

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