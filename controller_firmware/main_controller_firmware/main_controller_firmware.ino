/*
 * Author: Trevor Sherrard
 * Since: July 23, 2021
 * Purpose: firmware for main MCU on wireless 
 *          haptic controller.
 */
// include libraries needed for peripherals
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_DRV2605.h"

// include system libraries
#include "include/system_config.h"
#include "include/bsp.h"

// create adafruit BNO055 sensor handle
Adafruit_BNO055 bno = Adafruit_BNO055(-1, BNO_I2C_ADDR);

// create DRV2605 instance
Adafruit_DRV2605 drv;

// create bsp instance
hapticBSP bsp = hapticBSP(&bno, &drv);

// define global variables for sensor state data
bool is_BNO055_start;
bool is_DRV2605_start;

// define global variables for BNO055 Data
float euler_x, euler_y, euler_z;
float quat_x, quat_y, quat_z, quat_w;
int8_t temp;

// define flags for ISRs
bool was_in_top_button_ISR = false;
bool was_in_bottom_button_ISR = false;

// define button ISRs
void top_button_ISR()
{
  was_in_top_button_ISR = true;
}

void bottom_button_ISR()
{
  was_in_bottom_button_ISR = true;
}

void setup() {
  // set up serial communication
  Serial.begin(MAIN_SERIAL_BAUD);
  Serial1.begin(XBEE_SERIAL_BAUD);
  
  // start bsp components
  is_BNO055_start = bsp.startBNO055();
  is_DRV2605_start = bsp.startDRV2605();

  // setup up interrupt pins
  pinMode(INTERRUPT_PIN_1, INPUT_PULLUP);
  pinMode(INTERRUPT_PIN_2, INPUT_PULLUP);

  // attach ISR's to pins
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_1), top_button_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_2), bottom_button_ISR, FALLING);
}

void loop() {
  // address ISR flags if they are raised
  if(was_in_bottom_button_ISR)
  {
    Serial.println("Bottom Button Pressed!");
    bsp.playEffect(58);

    // reset flag
    was_in_bottom_button_ISR = false;
  }

  if(was_in_top_button_ISR)
  {
    Serial.println("Top Button Pressed!");
    bsp.playEffect(58);

    // reset flag
    was_in_top_button_ISR = false;
  }
  
  delay(BNO_LOOP_DELAY);
}
