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
#include <Adafruit_DRV2605.h>
#include <ArduinoJson.h>

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
int8_t temp;

// define flags for ISRs
bool was_in_top_button_ISR = false;
bool was_in_bottom_button_ISR = false;

// define a JSON document
StaticJsonDocument<200> JSON_doc;

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

  // set up json document with static values
  JSON_doc["controller_id"] = "right_hand";
  JSON_doc["euler_x"] = 0;
  JSON_doc["euler_y"] = 0;
  JSON_doc["euler_z"] = 0;

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

  // read from BNO055 sensor
  bsp.readEuler(euler_x, euler_y, euler_z);

  // update data in json doc
  JSON_doc["euler_x"] = euler_x;
  JSON_doc["euler_y"] = euler_y;
  JSON_doc["euler_z"] = euler_z;

  // serialize JSON
  serializeJson(JSON_doc, Serial1);
  Serial1.print('\n');
  
  delay(BNO_LOOP_DELAY);
}
