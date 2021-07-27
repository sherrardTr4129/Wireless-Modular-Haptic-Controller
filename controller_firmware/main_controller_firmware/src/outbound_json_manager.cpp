/*
 * Author: Trevor Sherrard
 * Since: July 26, 2021
 * Purpose: Class implementation for JSON document manager.
 */

#include <Arduino.h>
#include <ArduinoJson.h>

#include "../include/outbound_json_manager.h"

OutboundJsonDocManager::OutboundJsonDocManager(bool usingEuler, const char* controllerName)
{
   _usingEuler = usingEuler;
   _controllerName = controllerName;

   // set packet types
   _JSONDataDoc["packet_type"] = "outbound_data";
   _JSONEventDoc["packet_type"] = "button_event";

   // set controller IDs
   _JSONDataDoc["controller_id"] = _controllerName;
   _JSONEventDoc["controller_id"] = _controllerName;

   // set up euler template if using euler
   if(_usingEuler)
   {
      _JSONDataDoc["euler_x"] = 0;
      _JSONDataDoc["euler_y"] = 0;
      _JSONDataDoc["euler_z"] = 0;
   }

   // set up template using quaternion
   else
   {
      _JSONDataDoc["quat_x"] = 0;
      _JSONDataDoc["quat_y"] = 0;
      _JSONDataDoc["quat_z"] = 0;
      _JSONDataDoc["quat_w"] = 0;
   }

   // set up tempurature field
   _JSONDataDoc["temp_c"] = 0;

   // set up event template
   _JSONEventDoc["event_type"] = "";
}

void OutboundJsonDocManager::updateEuler(float x, float y, float z)
{
   if(_usingEuler)
   {
      _JSONDataDoc["euler_x"] = x;
      _JSONDataDoc["euler_y"] = y;
      _JSONDataDoc["euler_z"] = z;
   }
}

void OutboundJsonDocManager::updateQuat(float x, float y, float z, float w)
{
   if(!_usingEuler)
   {
      _JSONDataDoc["quat_x"] = x;
      _JSONDataDoc["quat_y"] = y;
      _JSONDataDoc["quat_z"] = z;
      _JSONDataDoc["quat_w"] = w;
   }
}

void OutboundJsonDocManager::updateTemp(int8_t temp)
{
   _JSONDataDoc["temp_c"] = temp;
}

void OutboundJsonDocManager::sendDataDoc()
{
   serializeJson(_JSONDataDoc, Serial1);
   Serial1.print('\n');
}

void OutboundJsonDocManager::topButtonEvent()
{
   _JSONEventDoc["event_type"] = "top_button_pressed";
}

void OutboundJsonDocManager::bottomButtonEvent()
{
   _JSONEventDoc["event_type"] = "bottom_button_pressed";
}

void OutboundJsonDocManager::sendEventDoc()
{
   serializeJson(_JSONEventDoc, Serial1);
   Serial1.print('\n');
}
