/*
 * Author: Trevor Sherrard
 * Since: July 25, 2021
 * Purpose: This file contains the code required
 *          to manage and drive the onboard controller 
 *          speaker.
 */
#include "Talkie.h"
#include "Vocab_US_Large.h"

// set up talkie instance
Talkie voice;

void arm_too_low()
{
  voice.say(sp2_DANGER);
  voice.say(sp2_POSITION);
  voice.say(sp2_IS);
  voice.say(sp4_TOO_LOW);
  voice.say(sp4_PLEASE);
  voice.say(sp5_RAISE);
}

void arm_too_high()
{
  voice.say(sp2_DANGER);
  voice.say(sp2_POSITION);
  voice.say(sp2_IS);
  voice.say(sp2_HIGH);
  voice.say(sp4_PLEASE);
  voice.say(sp4_LOWER);
}

void obstacle_close()
{
  voice.say(sp2_DANGER);
  voice.say(sp4_BASE);
  voice.say(sp4_CONTACT);
  voice.say(sp4_CLOSE);
  voice.say(sp4_PLEASE);
  voice.say(sp2_CHANGE);
  voice.say(sp4_COURSE);
  voice.say(sp4_IMMEDIATELY);
}
void setup() {  

}

void loop() {
  arm_too_high();
  delay(500);
  obstacle_close();
  

}
