#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include "MotorsQuad.h"
#include "ICM20948.h"

#include <stdint.h>

// calibration data structure
typedef struct {
	// gyroscope offsets in full scale format
	int16_t offset_gx_1000dps, offset_gy_1000dps, offset_gz_1000dps;
	// accelerometer offsets in full scale format
	int16_t offset_ax_32g, offset_ay_32g, offset_az_32g;
	// magnetometer hard iron distortion correction
	float offset_mx, offset_my, offset_mz;
	// magnetometer soft iron distortion correction
	float scale_mx, scale_my, scale_mz;
} calibration_data;

// calibrate gyroscope, accelerometer or magnetometer on rc command and return true if any calibration was performed
bool calibration(MotorsQuad &motors, ICM20948_SPI &imu, uint16_t *rc_channelValue, calibration_data &calibration_eeprom);

#endif