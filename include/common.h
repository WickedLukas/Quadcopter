#ifndef COMMON_H_
#define COMMON_H_

#include <stdint.h>

// print debug outputs through serial
//#define DEBUG

// plot through Processing
//#define PLOT

// send imu data through serial (for example to visualize it in "Processing")
//#define SEND_SERIAL

// ! For safety, define MOTORS_OFF to prevent motors from running.
//#define MOTORS_OFF

// define which features shall be used
#define USE_BAR   // use barometer
#define USE_MAG   // use magnetometer // TODO: Check if magnetometer data is reliable when motors are running.
#define USE_GPS   // use GPS
#define USE_SDLOG // use SD card for data logging

#ifdef DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINT2(x, y) Serial.print(x, y)
    #define DEBUG_PRINTLN2(x, y) Serial.println(x, y)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINT2(x, y)
    #define DEBUG_PRINTLN2(x, y)
#endif

// macros to help with data logging to SD card
#define SD_LOG(id_value) sdCardLogger.log(logId::id_value, id_value);
#define SD_LOG2D(id_value, digits) sdCardLogger.log(logId::id_value, id_value, digits);
#define SD_LOG2(id, value) sdCardLogger.log(logId::id, value);
#define SD_LOG3(id, value, digits) sdCardLogger.log(logId::id, value, digits);

#define rcPort Serial3
#define gpsPort Serial5

// address for eeprom data
#define ADDRESS_EEPROM 64

// LED pin
#define LED_PIN 23

// rc channel assignment
#define ROLL 0
#define PITCH 1
#define YAW 3
#define THROTTLE 2
#define ARM 6	// disarm: 1000, enable arming: 2000
#define FMODE 8 // flight mode
#define RTL 9	// return to launch

// Stores the errors which occurred and disables arming.
// A restart is required to reset the status and enable arming.
extern uint8_t error_code;

// imu interrupt
extern volatile bool imuInterrupt;

// conversion factors between radians and degrees
const float DEG_PER_RAD = (float)4068 / 71;
const float RAD_PER_DEG = (float)71 / 4068;

// update LED
// mode: 0		off
// mode: 1		blink
// mode: > 1	on
void updateLed(uint8_t pin, uint8_t mode, uint32_t interval_ms = 1000);

// adjust angle range to [min, max)
void adjustAngleRange(float min, float max, float &angle);

// transform quaternion from body- to ned-frame using the pose quaternion
void body2nedFrame(const float *pose_q, const float *q_body, float *q_ned_result);

// multiply two quaternions (Hamilton product)
void qMultiply(const float *q1, const float *q2, float *result_q);

#endif