/*
 * Author: Trevor Sherrard
 * Since: July 25, 2021
 * Purpose: This file contains the code required
 *          to manage and drive the onboard controller 
 *          speaker.
 */
 
// include TTS and peripheral libraries 
#include "Talkie.h"
#include "Vocab_US_Large.h"
#include "Vocab_Special.h"
#include <Wire.h>

// include system libraries
#include "include/system_config.h"
#include "include/speech_driver.h"

// create speech driver object
Talkie talkie;
speechDriver spk_drv(&talkie);

// create callback for i2c events
void handle_i2c_event()
{
  // read control byte
  int rx_byte = Wire.read();

  // switch on byte, execute proper event
  switch(rx_byte)
  {
    case 0:
      spk_drv.arm_too_low();
      break;
    case 1:
      spk_drv.arm_too_high();
      break;
    case 2:
      spk_drv.obstacle_close();
      break;
    default:
      break;
  } 
}

void setup() {
  Wire.begin(I2C_ADDR); // join i2c bus
  Wire.onReceive(handle_i2c_event);
}

void loop() {
  delay(SPK_LOOP_REFRESH);
}
