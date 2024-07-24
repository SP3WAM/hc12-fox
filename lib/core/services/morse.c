#include <Arduino.h>
#include "morse.h"
#include "services/fake_f3e.h"

#define DOT_DURATION_MILLIS  150
#define DASH_DURATION_MILLIS 3 * DOT_DURATION_MILLIS

void sendMOE()
{
    Serial_println_s("sending MOE...");

    // send M
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(3 * DOT_DURATION_MILLIS);
    
    // send O
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(3 * DOT_DURATION_MILLIS);

    // send E
    sendDot();
    delay(7 * DOT_DURATION_MILLIS);
}

void sendMOI()
{
    Serial_println_s("sending MOI...");

    // send M
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(3 * DOT_DURATION_MILLIS);
    
    // send O
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(3 * DOT_DURATION_MILLIS);

    // send I
    sendDot();
    delay(DOT_DURATION_MILLIS);  
    sendDot();
    delay(7 * DOT_DURATION_MILLIS);
}

void sendMOS()
{
    Serial_println_s("sending MOS...");

    // send M
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(3 * DOT_DURATION_MILLIS);
    
    // send O
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(3 * DOT_DURATION_MILLIS);

    // send S
    sendDot();
    delay(DOT_DURATION_MILLIS); 
    sendDot();
    delay(DOT_DURATION_MILLIS);  
    sendDot();
    delay(7 * DOT_DURATION_MILLIS);
}

void sendMOH()
{
    Serial_println_s("sending MOH...");

    // send M
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(3 * DOT_DURATION_MILLIS);
    
    // send O
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(3 * DOT_DURATION_MILLIS);

    // send H
    sendDot();
    delay(DOT_DURATION_MILLIS); 
    sendDot();
    delay(DOT_DURATION_MILLIS); 
    sendDot();
    delay(DOT_DURATION_MILLIS);  
    sendDot();
    delay(7 * DOT_DURATION_MILLIS);
}

void sendMO5()
{
    Serial_println_s("sending MO5...");

    // send M
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(3 * DOT_DURATION_MILLIS);
    
    // send O
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(DOT_DURATION_MILLIS);  
    sendDash();
    delay(3 * DOT_DURATION_MILLIS);

    // send 5
    sendDot();
    delay(DOT_DURATION_MILLIS); 
    sendDot();
    delay(DOT_DURATION_MILLIS); 
    sendDot();
    delay(DOT_DURATION_MILLIS); 
    sendDot();
    delay(DOT_DURATION_MILLIS);  
    sendDot();
    delay(7 * DOT_DURATION_MILLIS);
}

void inline sendDot()
{
    fake_f3e_tone(700, ((unsigned long)DOT_DURATION_MILLIS) * ((unsigned long)1000));
}

void inline sendDash()
{
    fake_f3e_tone(700, ((unsigned long)DASH_DURATION_MILLIS) * ((unsigned long)1000));
}