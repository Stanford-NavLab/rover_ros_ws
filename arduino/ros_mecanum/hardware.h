#include <Arduino.h>

// Define LED pins //
#define L1 44			// Output Status LEDs
#define L2 42
#define L3 40
#define L4 38

// PWM Motor Controller Pins //
#define pwmFRpin 9
#define dirFRpinA 24
#define dirFRpinB 22
//#define brkA 1
#define pwmFLpin 5
#define dirFLpinA 28
#define dirFLpinB 26
//#define brkB 8
#define pwmRRpin 6
#define dirRRpinA 7
#define dirRRpinB 8
//#define brkC 11
#define pwmRLpin 3
#define dirRLpinA 2
#define dirRLpinB 4

//#define pwmFRpin 9
//#define dirFRpinA 24
//#define dirFRpinB 22
////#define brkA 1
//#define pwmFLpin 5
//#define dirFLpinA 26
//#define dirFLpinB 28
////#define brkB 8
//#define pwmRRpin 6
//#define dirRRpinA 7
//#define dirRRpinB 8
////#define brkC 11
//#define pwmRLpin 3
//#define dirRLpinA 4
//#define dirRLpinB 2

// Battery Monitoring Analog Input //
#define batt24VInput 1	//analog pin 1

// Output signals for TTL Relays //
#define output1 41		
#define output2 43
#define output3 45
#define output4 47

#define strafePinRC 11
#define drivePinRC 13
#define turnPinRC 12

#define NEUTRAL_DRIVE 1500
#define NEUTRAL_TURN 1500
#define NEUTRAL_STRAFE 1500
