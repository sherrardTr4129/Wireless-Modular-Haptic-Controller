#ifndef SPEECH_DRV
#define SPEECH_DRV

// define speechDriver class
class speechDriver
{
   public:
      speechDriver(Talkie *voice);
      void arm_too_low();
      void arm_too_high();
      void obstacle_close();
   private:
      Talkie *_voice;
};

#endif

