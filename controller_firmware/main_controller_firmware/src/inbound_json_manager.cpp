/*
 * Author: Trevor Sherrard
 * Since: July 27, 2021
 * Purpose: Class implementation for inbound JSON document manager.
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include "../include/inbound_json_manager.h"

/*
 * Default class constructor.
 *
 * params:
 * 	controllerName -> name of this controller instance
 *
 * returns:
 * 	None
 */
InboundJsonDocManager::InboundJsonDocManager(const char* controllerName)
{
   _controllerName = controllerName;
   _voiceActionID = -1;
   _hapticActionID = -1;
}

/*
 * Process inbound JSON data. only keep data if it matches this controllers
 * controller_name field.
 *
 * params:
 * 	json -> char array containing recieved JSON
 * return:
 * 	parse_status -> indication of JSON parsing success or failure
 */
bool InboundJsonDocManager::procInboundDoc(char json[])
{
   // attempt to deserialize
   DeserializationError error = deserializeJson(_JSON_inbound_doc, json);

   // exit early if something went wrong
   if(error)
   {
      Serial.println("Inbound JSON: Cold Not Parse JSON!");
      Serial.println(error.f_str());
      return false;
   }

   // otherwise extract data
   else
   {
      // make sure data was meant for this controller.
      const char* tempName = _JSON_inbound_doc["controller_name"];
      if(strcmp(_controllerName, tempName) == 0)
      {
	 // extract data and populate internal fields
         _voiceActionID = _JSON_inbound_doc["voice_action_id"];
         _hapticActionID = _JSON_inbound_doc["haptic_action_id"];
	 return true;
      }
      else
      {
	 // if not meant for this controller, exit.
         return false;
      }
   }
}

/*
 * returns internal voice action ID value
 *
 * params:
 * 	None
 * returns:
 * 	_voiceActionID -> internal voice action ID
 */
int InboundJsonDocManager::getVoiceActionID()
{
   return _voiceActionID;
}

/*
 * returns internal haptic action ID value
 *
 * params:
 * 	None
 * returns:
 * 	_hapticActionID -> internal haptic action ID
 */
int InboundJsonDocManager::getHapticActionID()
{
   return _hapticActionID;
}
