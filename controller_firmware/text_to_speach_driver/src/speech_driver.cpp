/*
 * Author: Trevor Sherrard
 * Since: July 26, 2021
 * Purpose: class implementation for TTS
 */

// include libraries for TTS
#include "Arduino.h"
#include "Talkie.h"
#include "Vocab_US_Large.h"
#include "Vocab_Special.h"
#include "Vocab_US_TI99.h"

// include system libraries
#include "../include/speech_driver.h"

speechDriver::speechDriver(Talkie *voice)
{
   // set up talkie pointer
   _voice = voice;
}

void speechDriver::course_instructions()
{
   _voice->say(spt_START);
   _voice->say(spt_COURSE);
   _voice->say(spt_COMMAND);
}

void speechDriver::fine_instructions()
{
   _voice->say(spt_START);
   _voice->say(spt_FINE);
   _voice->say(spt_COMMAND);
}

void speechDriver::arm_too_low()
{
   _voice->say(sp2_DANGER);
   _voice->say(sp2_POSITION);
   _voice->say(sp2_IS);
   _voice->say(sp4_TOO_LOW);
   _voice->say(sp4_PLEASE);
   _voice->say(sp5_RAISE);
   _voice->say(spPAUSE1);
}

void speechDriver::arm_too_high()
{
   _voice->say(sp2_DANGER);
   _voice->say(sp2_POSITION);
   _voice->say(sp2_IS);
   _voice->say(sp2_HIGH);
   _voice->say(sp4_PLEASE);
   _voice->say(sp4_LOWER);
   _voice->say(spPAUSE1);
}

void speechDriver::obstacle_close()
{
   _voice->say(sp2_DANGER);
   _voice->say(sp4_BASE);
   _voice->say(sp4_CONTACT);
   _voice->say(sp4_CLOSE);
   _voice->say(sp4_PLEASE);
   _voice->say(sp2_CHANGE);
   _voice->say(sp4_COURSE);
   _voice->say(sp4_IMMEDIATELY);
   _voice->say(spPAUSE1);
}
