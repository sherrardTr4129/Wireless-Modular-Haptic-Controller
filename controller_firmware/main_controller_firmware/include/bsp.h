/*
 * Author: Trevor Sherrard
 * Since: July 23, 2021
 * Purpose: Class definition for haptic controller bsp. 
 */

class hapticBSP
{
   public:
	   hapticBSP(Adafruit_BNO055 *bno, Adafruit_DRV2605 *drv);

	   //BNO055 methods
	   bool startBNO055();
	   void readEuler(float &x, float &y, float &z);
	   void readQuat(float &x, float &y, float &z, float &w);
	   void readTemp(int8_t &temp);

	   // DRV2605 methods
	   bool startDRV2605();
	   void playEffect(uint8_t effect); 
   private:
	   Adafruit_BNO055 *_bno;
	   Adafruit_DRV2605 *_drv;
};
