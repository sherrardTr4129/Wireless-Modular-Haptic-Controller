/*
 * Author: Trevor Sherrard
 * Since: July 27, 2021
 * Purpose: Class definition for inbound JSON document manager.
 */

#ifndef INBOUND_JSON
#define INBOUND_JSON

#define INBOUND_BUFFER_SIZE	100

class InboundJsonDocManager
{
   public:
	   InboundJsonDocManager(const char* controllerName);
	   bool procInboundDoc(char json[]);
	   int getVoiceActionID();
	   int getHapticActionID();
   private:
	   StaticJsonDocument<100> _JSON_inbound_doc;
	   const char* _controllerName;
	   uint8_t _voiceActionID;
	   uint8_t _hapticActionID;
};

#endif
