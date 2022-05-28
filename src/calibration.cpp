#include "calibration.h"
#include "common.h"

#include <Arduino.h>
#include <EEPROM.h>

// calibrate gyroscope, accelerometer or magnetometer on rc command and return true if any calibration was performed
bool calibration(MotorsQuad &motors, ICM20948_SPI &imu, uint16_t* rc_channelValue, calibration_data &calibration_eeprom) {
	static uint32_t t_imuCalibration;
	t_imuCalibration = micros();
	
	static uint32_t t_calibrateGyro = 0, t_calibrateAccel = 0, t_calibrateMag = 0;
	if (motors.getState() == MotorsQuad::State::disarmed) {
		if ((rc_channelValue[PITCH] < 1100) && (rc_channelValue[ROLL] > 1400) && (rc_channelValue[ROLL] < 1600)) {
			if ((rc_channelValue[THROTTLE] < 1100) && (rc_channelValue[YAW] < 1100)) {
				// hold right stick bottom center and left stick bottom-left to start gyro calibration	(2s)
				t_calibrateAccel = 0;
				t_calibrateMag = 0;
				if (t_calibrateGyro == 0) {
					t_calibrateGyro = t_imuCalibration;
					return false;
				}
				else if ((t_imuCalibration - t_calibrateGyro) > 2000000) {
					// turn on LED to indicate calibration
					updateLED(LED_PIN, 2);
					imu.calibrate_gyro(imuInterrupt, 5.0, 1, calibration_eeprom.offset_gx_1000dps, calibration_eeprom.offset_gy_1000dps, calibration_eeprom.offset_gz_1000dps);
					EEPROM.put(ADDRESS_EEPROM, calibration_eeprom);
					t_calibrateGyro = 0;
					// turn off LED
					updateLED(LED_PIN, 0);
					return true;
				}
				else {
					return false;
				}
			}
			else if ((rc_channelValue[THROTTLE] > 1900) && (rc_channelValue[YAW] < 1100)) {
				// hold right stick bottom center and left stick top-left to start accel calibration	(2s)
				t_calibrateGyro = 0;
				t_calibrateMag = 0;
				if (t_calibrateAccel == 0) {
					t_calibrateAccel = t_imuCalibration;
					return false;
				}
				else if ((t_imuCalibration - t_calibrateAccel) > 2000000) {
					// turn on LED to indicate calibration
					updateLED(LED_PIN, 2);
					imu.calibrate_accel(imuInterrupt, 5.0, 16, calibration_eeprom.offset_ax_32g, calibration_eeprom.offset_ay_32g, calibration_eeprom.offset_az_32g);
					EEPROM.put(ADDRESS_EEPROM, calibration_eeprom);
					t_calibrateAccel = 0;
					// turn off LED
					updateLED(LED_PIN, 0);
					return true;
				}
				else {
					return false;
				}
			}
			else if ((rc_channelValue[THROTTLE] > 1900) && (rc_channelValue[YAW] > 1900)) {
				// hold right stick bottom center and left stick top-right to start mag calibration	(2s)
				t_calibrateGyro = 0;
				t_calibrateAccel = 0;
				if (t_calibrateMag == 0) {
					t_calibrateMag = t_imuCalibration;
					return false;
				}
				else if ((t_imuCalibration - t_calibrateMag) > 2000000) {
					// turn on LED to indicate calibration
					updateLED(LED_PIN, 2);
					imu.calibrate_mag(imuInterrupt, 60, 500, calibration_eeprom.offset_mx, calibration_eeprom.offset_my, calibration_eeprom.offset_mz, calibration_eeprom.scale_mx, calibration_eeprom.scale_my, calibration_eeprom.scale_mz);
					EEPROM.put(ADDRESS_EEPROM, calibration_eeprom);
					t_calibrateMag = 0;
					// turn off LED
					updateLED(LED_PIN, 0);
					return true;
				}
				else {
					return false;
				}
			}
		}
	}
	
	t_calibrateGyro = 0;
	t_calibrateAccel = 0;
	t_calibrateMag = 0;
	
	return false;
}