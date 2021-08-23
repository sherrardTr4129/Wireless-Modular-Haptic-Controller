# Controller Firmware Design
This sub-directory contains the firmware developed for the two processors on the hand-held controller. Please see below for notes about each MCU's firmware and the procedure to flash the firmware onto each of the MCU's. 

## Firmware Dependencies 
This section will breifly outline the different firmware dependencies within this project. Please note that all firmware dependencies are obtainable through the arduino software manager. 

### Main Controller Firmware Dependencies
* **Adafruit Sensor Library** - This library provides general functionallity required for the two onboard adafruit sensors.
* **Adafruit BNO055 Library** - This library provides functionallity needed to read data from the onboard BNO055 sensor via an I2C interface. 
* **Adafruit DRV2605 Library** - This library provides functionallity needed to interface with the DRV2605 driver board via an I2C interface.
* **ArduinoJson** - This library provides JSON document construction functionallity and functionallity to serialize said documents via the XBee wireless interface. 

### Text-To-Speech Controller Firmware Dependencies
* **Talkie** - This library provides rudimentary TTS functionallity utilizing the onboard MCU TIMER1 and TIMER2 peripherals. 

## Main Controller Firmware
The main controller firmware is designed to run on the ATMEGA2560 processor on the PCB. Its main purpose is to actuate audio-haptic displays based on requests received by the basestation, and to transmit data from the onboard BNO055 sensor back to the basestation. Outside of the main\_controller\_firmware.ino file, two other software packages are provided. The first is the BSP, which provides functionallity required to initialize and manage all onboard actuators and sensors. The second is the inbound and outbound data stream managers. These classes are designed to unpack incomming request JSON packets from the basestation and package and serialize BNO055 data JSON packets respectively. 

For each controller on the network, two lines of code need to be examined and potentially changed at compile time within the main\_controller\_firmware.ino file. The first is:

```C
bool usingEuler = true;
```
This boolean flag will setup the BNO55 sensor in euler angle mode if true, and quaternion mode if false. The outbound JSON document will be set up differently depending on the state of this variable as well. The second line to examine is:

```C
const char* controller_name = "right_hand";
```
This line defines the given controller's name. The controller name is an important field in all inbound and outbound JSON packets. For outbound data, the controller\_name field is used to determine the origin of the recieved BNO055 data packet. For inbound data, the recieved data packet needs to have the controller\_name field value equivalent to the name set in the above line of code for it to be processed by a given controller. Please ajdust this line accordingly based on the desired HRI configuration.

## Text-To-Speech Controller Firmware
The Text-To-Speech controller firmware is designed to be run on the ATMEGA328P-AU processor on the PCB. It's main purpose is to accept requests to play pre-rendered TTS audio events via the I2C interface managed by the main controller. The I2C transaction is simple, as it requires a single integer number between 0 and 4 corresponding to one of the aforementioned TTS events. 

## Bootloader and Firmware Flashing Proceedure
The pins on the PCB itself needed to flash the bootloader and respective firmware can be seen highlighted in the image below:

![MCU Bootloader and Firmware Flashing Pins](../documentation/PCB_Prog_Pins.PNG)

## XBee Radio Configuration
