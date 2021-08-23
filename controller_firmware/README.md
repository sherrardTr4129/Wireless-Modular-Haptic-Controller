# Controller Firmware Design
This sub-directory contains the firmware developed for the two processors on the hand-held controller. Please see below for notes about each MCU's firmware and the procedure to flash the firmware onto each of the MCU's. 

## Firmware Dependencies 
This section will breifly outline the different firmware dependencies within this project. Please note that all firmware dependencies are obtainable through the arduino software manager. 

### Main Controller Firmware Dependencies
* Adafruit\_Sensor Library - This library provides general functionallity required for the two onboard adafruit sensors.
* Adafruit\_BNO055 Library - This library provides functionallity needed to read data from the onboard BNO055 sensor via an I2C interface. 
* Adafruit\_DRV2605 Library - This library provides functionallity needed to interface with the DRV2605 driver board via an I2C interface.
* ArduinoJson - This library provides JSON document construction functionallity and functionallity to serialize said documents via the XBee wireless interface. 

### Text-To-Speech Controller Firmware Dependencies
* Talkie - This library provides rudimentary TTS functionallity utilizing the onboard MCU TIMER1 and TIMER2 peripherals. 

## Main Controller Firmware

## Text-To-Speech Controller Firmware

## Bootloader and Firmware Flashing Proceedure

## XBee Radio Configuration
