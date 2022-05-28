#ifndef COMMON_H_
#define COMMON_H_

#include <stdint.h>

// print debug outputs through serial
//#define DEBUG

#ifdef DEBUG
	#define DEBUG_PRINT(x) Serial.print(x)
	#define DEBUG_PRINTLN(x) Serial.println(x)
	#define DEBUG_PRINT2(x,y) Serial.print(x,y)
	#define DEBUG_PRINTLN2(x,y) Serial.println(x,y)
#else
	#define DEBUG_PRINT(x)
	#define DEBUG_PRINTLN(x)
	#define DEBUG_PRINT2(x,y)
	#define DEBUG_PRINTLN2(x,y)
#endif

// address for eeprom data
#define ADDRESS_EEPROM 128

// LED pin
#define LED_PIN 23

// rc channel assignment
#define ROLL		0
#define PITCH		1
#define YAW			3
#define THROTTLE	2
#define ARM			6	// disarm: 1000, enable arming: 2000
#define FMODE		8

// Stores the errors which occurred and disables arming.
// A restart is required to reset the error and enable arming.
extern uint8_t error_code;

// imu interrupt
extern volatile bool imuInterrupt;

// update LED
// mode: 0		off
// mode: 1		blink
// mode: > 1	on
void updateLED(uint8_t pin, uint8_t mode, uint32_t interval_ms = 1000);

#endif