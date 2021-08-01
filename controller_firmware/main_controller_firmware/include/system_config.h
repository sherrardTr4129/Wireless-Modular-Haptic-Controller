#ifndef SYS_CONFIG
#define SYS_CONFIG

// define interrupt pins
#define INTERRUPT_PIN_1		2
#define INTERRUPT_PIN_2		3
#define MIN_MS_BETWEEN_INT	300

// define BNO055 constants
#define BNO_I2C_ADDR		0x28
#define BNO_STARTUP_DELAY_MS	250
#define BNO_LOOP_DELAY		100

//define Serial parameters
#define MAIN_SERIAL_BAUD	115200
#define XBEE_SERIAL_BAUD	9600

// define DRV2605 parameters
#define EFFECT_DELAY		100
#define MAX_HAPTIC_EFFECT	117
#define PULSE_STRONG		52
#define CLICK_STRONG		17
#define BUZZ_STRONG		14
#define ONE_SEC_ALERT		16
#define SMOOTH_MEDIUM_RAMP	85
#define STRONG_RAMP		58

// define enum for TTS control
enum tts_control 
{
	ARM_TOO_LOW,
	ARM_TOO_HIGH,
	COLLISION_CLOSE,
	COURSE_SCALING,
	FINE_SCALING,
	MAX_TTS_ENUM_VAL
};

#define TTS_I2C_ADDR		8

#endif
