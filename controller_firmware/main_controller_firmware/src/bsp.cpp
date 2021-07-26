/*
 * Author: Trevor Sherrard
 * Since: July 23, 2021
 * Purpose: Class implementation for haptic controller bsp.
 */

// include peripheral libraries
#include "Arduino.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_DRV2605.h"
#include <Wire.h>

// include system libraries
#include "../include/system_config.h"
#include "../include/bsp.h"


hapticBSP::hapticBSP(Adafruit_BNO055 *bno, Adafruit_DRV2605 *drv)
{
   _bno = bno;
   _drv = drv;
}

bool hapticBSP::startBNO055()
{
   // start bno sensor
   bool start_status = _bno->begin();
   if(!start_status)
   {
      Serial.println("BSP: No BNO055 detected!");
   }
   else
   {
      Serial.println("BSP: Started BNO055!");

      //configure sensor
      delay(BNO_STARTUP_DELAY_MS);
      _bno->setExtCrystalUse(true);
   }

   return start_status;
}

void hapticBSP::readEuler(float &x, float &y, float &z)
{
   // get vector from sensor
   imu::Vector<3> euler = _bno->getVector(Adafruit_BNO055::VECTOR_EULER);
   
   // extract sensor data
   x = euler.x();
   y = euler.y();
   z = euler.z();
}

void hapticBSP::readQuat(float &x, float &y, float &z, float &w)
{
   // get quaternion from sensor
   imu::Quaternion quat = _bno->getQuat();

   // extract components
   x = quat.x();
   y = quat.y();
   z = quat.z();
   w = quat.w();
}

void hapticBSP::readTemp(int8_t &temp)
{
   // grab temp from sensor
   temp = _bno->getTemp();
}

bool hapticBSP::startDRV2605()
{
   // try to start sensor
   bool start_status = _drv->begin();
   if(!start_status)
   {
      Serial.println("BSP: Could Not Start DRV2605!");
      while(1);
   }
   else
   {
      Serial.println("BSP: Started DRV2605!");

      // configure sensor
      _drv->selectLibrary(1);
      _drv->setMode(DRV2605_MODE_INTTRIG);
   }

   return start_status;
}

void hapticBSP::playEffect(uint8_t effect)
{
   // bound effect numbering
   if(effect == 0)
   {
      effect = 1;  
   }
   else if(effect > MAX_HAPTIC_EFFECT)
   {
      effect = MAX_HAPTIC_EFFECT;
   }
   // select effect
   _drv->setWaveform(0, effect);
   _drv->setWaveform(1, 0);

   // play effect
   _drv->go();

   // wait for effect to finish
   delay(EFFECT_DELAY);
}

void hapticBSP::setup_tts_i2c()
{
   Wire.begin();
}

void hapticBSP::speak_tts(uint8_t spk_val)
{
   // constrain spk_val
   if(spk_val > 2)
   {
      spk_val = 2;
   }

   // start transmission
   Wire.beginTransmission(TTS_I2C_ADDR);

   // write byte
   Wire.write(spk_val);

   // end transmission
   Wire.endTransmission();
}
