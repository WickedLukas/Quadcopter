#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include "motorsQuad.h"
#include "ICM20948.h"

#include <stdint.h>

// calibration data structure
typedef struct {
	// gyroscope offsets in full scale format
	int16_t gyroOffset_1000dps_xyz[3];
	// accelerometer offsets in full scale format
	int16_t accelOffset_32g_xyz[3];
	// magnetometer hard iron distortion correction
	float magOffset_xyz[3];
	// magnetometer soft iron distortion correction
	float magScale_xyz[3];
} calibration_data;

// calibrate gyroscope, accelerometer or magnetometer on rc command and return true if any calibration was performed
bool calibration(MotorsQuad &motors, ICM20948_SPI &imu, uint16_t *rc_channelValue, calibration_data &calibration_eeprom);

#endif