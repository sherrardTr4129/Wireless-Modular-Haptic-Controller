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
#include "include/outbound_json_manager.h"
#include "include/inbound_json_manager.h"

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

// define outbound JSON doc manager
bool usingEuler = true;
const char* controller_name = "right_hand";
OutboundJsonDocManager outbound_doc = OutboundJsonDocManager(usingEuler, controller_name);

// define inbound JSON doc manager
InboundJsonDocManager inbound_doc = InboundJsonDocManager(controller_name);
String inbound_data_str;
char inbound_data[INBOUND_BUFFER_SIZE];

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
  bsp.setup_tts_i2c();

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
    // update doc with event info and serialize
    outbound_doc.bottomButtonEvent();
    outbound_doc.sendEventDoc();

    // play haptic effect and TTS
    bsp.playEffect(STRONG_RAMP);
    bsp.speak_tts(ARM_TOO_LOW);

    // reset flag
    was_in_bottom_button_ISR = false;
  }

  if(was_in_top_button_ISR)
  {
    // update doc with event info and serialize
    outbound_doc.topButtonEvent();
    outbound_doc.sendEventDoc();

    // play haptic effect and TTS
    bsp.playEffect(STRONG_RAMP);
    bsp.speak_tts(ARM_TOO_HIGH);

    // reset flag
    was_in_top_button_ISR = false;
  }

  // see if we have new incoming data from the Xbee
  Serial1.available();
  if(Serial1.available() > 0)
  {
    // if so, read the data from Serial1
    inbound_data_str = Serial1.readString();

    // move string to char array
    inbound_data_str.toCharArray(inbound_data, INBOUND_BUFFER_SIZE);

    // try to parse new data
    bool isParsed = inbound_doc.procInboundDoc(inbound_data);

    // extract values from object
    if (isParsed)
    {
      int voiceVal = inbound_doc.getVoiceActionID();
      int hapticVal = inbound_doc.getHapticActionID();

      // if hapticVal or voiceVal are non-zero, perform
      // respective audio or haptic display
      if(hapticVal >= 0)
      {
         bsp.playEffect(hapticVal);
      }

      if(voiceVal >= 0)
      {
        bsp.speak_tts(voiceVal);
      }
    }
  }

  // read from BNO055 sensor
  bsp.readEuler(euler_x, euler_y, euler_z);
  bsp.readTemp(temp);

  outbound_doc.updateEuler(euler_x, euler_y, euler_z);
  outbound_doc.updateTemp(temp);
  outbound_doc.sendDataDoc();
  
  delay(BNO_LOOP_DELAY);
}
