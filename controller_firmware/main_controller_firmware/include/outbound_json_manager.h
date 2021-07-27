/*
 * Author: Trevor Sherrard
 * Since: July 26, 2021
 * Purpose: Class definition for JSON document manager.
 */

#ifndef OUTBOUND_JSON
#define OUTBOUND_JSON

class OutboundJsonDocManager
{
   public:
	 // methods for BNO055 data management
	 OutboundJsonDocManager(bool usingEuler, const char* controllerName);
	 void updateEuler(float x, float y, float z);
	 void updateQuat(float x, float y, float z, float w);
	 void updateTemp(int8_t temp);
	 void sendDataDoc();

	 // methods for button interrupt data transmission
	 void topButtonEvent();
	 void bottomButtonEvent();
	 void sendEventDoc();

   private:
	 StaticJsonDocument<200> _JSONDataDoc;
	 StaticJsonDocument<100> _JSONEventDoc;
	 const char* _controllerName;
	 bool _usingEuler;
};

#endif
