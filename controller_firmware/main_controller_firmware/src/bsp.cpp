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

/*
 * constructor for BSP.
 *
 * params:
 * 	bno -> pointer to Adafruit_BNO055 instance
 * 	drv -> pointer to Adafruit_DRV2605 instance
 */
hapticBSP::hapticBSP(Adafruit_BNO055 *bno, Adafruit_DRV2605 *drv)
{
   _bno = bno;
   _drv = drv;
}

/*
 * function to set up BNO055 sensor. Needs to 
 * be called in void setup on controller startup.
 *
 * params:
 * 	None
 * returns:
 * 	bool -> An indication of the
 * 		sensor set up status.
 */
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

/*
 * function to read euler angle values from BNO055
 * sensor. Obtained values are populated into variables
 * passed in by reference. NOTE: angles are read in degrees
 *
 * params:
 * 	&x,&y,&z -> angles to populate with sensor data
 * returns:
 * 	void
 */
void hapticBSP::readEuler(float &x, float &y, float &z)
{
   // get vector from sensor
   imu::Vector<3> euler = _bno->getVector(Adafruit_BNO055::VECTOR_EULER);
   
   // extract sensor data
   x = euler.x();
   y = euler.y();
   z = euler.z();
}

/*
 * function to read quaternion values from BNO055
 * sensor. Obtained values are populated into variables
 * passed in by reference.
 *
 * params:
 *      &x,&y,&z,&w -> quaternion components to populate with sensor data
 * returns:
 *      void
 */

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

/*
 * Function to read tempurature from BNO055 sensor. 
 * Obtained value is populated into variable passed in by
 * reference. NOTE: temp is in degrees C.
 *
 * params:
 * 	&temp -> temp value to populate with sensor data.
 * returns:
 * 	void
 */
void hapticBSP::readTemp(int8_t &temp)
{
   // grab temp from sensor
   temp = _bno->getTemp();
}

/*
 * This function is used to start the DRV2605 sensor
 * as an i2c minion.
 *
 * params:
 * 	None
 * returns:
 * 	bool -> a boolean indicating the status of
 * 		the sensor initialization.
 */
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

/*
 * This function allows the user to play one of the pre-rendered
 * haptic displays from the DRV2605 sensor. 
 *
 * params:
 *	effect -> integer from one to 117 representing the haptic effect
 *		  to play. Please refer to the table in link below for
 *		  effect number to effect mapping
 *		  Link: https://learn.adafruit.com/assets/72593
 * returns:
 * 	void 
 */
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

/*
 * This function initalizes the main controller as 
 * an i2c host for use with the TTS processor. 
 *
 * params:
 * 	None
 * returns:
 * 	void
 */
void hapticBSP::setup_tts_i2c()
{
   Wire.begin();
}

/*
 * This function sends a control byte to the TTS
 * processor to actuate a voice prompt.
 *
 * params:
 * 	spk_val -> voice effect number to play 
 * 	over TTS. Please see TTS processor code
 * 	for spk_val -> voice mapping.
 *
 * returns:
 * 	void 
 */
void hapticBSP::speak_tts(uint8_t spk_val)
{
   // constrain spk_val
   if(spk_val > MAX_TTS_ENUM_VAL - 1)
   {
      spk_val = MAX_TTS_ENUM_VAL - 1;
   }

   // start transmission
   Wire.beginTransmission(TTS_I2C_ADDR);

   // write byte
   Wire.write(spk_val);

   // end transmission
   Wire.endTransmission();
}
