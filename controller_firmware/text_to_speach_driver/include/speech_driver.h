/*
 * Author: Trevor Sherrard
 * Since: July 26, 2021
 * Purpose: class definition for TTS
 */

#ifndef SPEECH_DRV
#define SPEECH_DRV

// define speechDriver class
class speechDriver
{
   public:
      speechDriver(Talkie *voice);
      void course_instructions();
      void fine_instructions();
      void arm_too_low();
      void arm_too_high();
      void obstacle_close();
   private:
      Talkie *_voice;
};

#endif

