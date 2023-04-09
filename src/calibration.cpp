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
		if ((rc_channelValue[PITCH] < 1050) && (rc_channelValue[ROLL] > 1450) && (rc_channelValue[ROLL] < 1550)) {
			if ((rc_channelValue[THROTTLE] < 1050) && (rc_channelValue[YAW] < 1050)) {
				// hold right stick bottom center and left stick bottom-left to start gyro calibration	(2s)
				t_calibrateAccel = 0;
				t_calibrateMag = 0;
				if (t_calibrateGyro == 0) {
					t_calibrateGyro = t_imuCalibration;
					return false;
				}
				else if ((t_imuCalibration - t_calibrateGyro) > 2000000) {
					updateLed(LED_PIN, 2); // turn on LED to indicate calibration
					imu.calibrate_gyro(imuInterrupt, 5.0, 1, calibration_eeprom.gyroOffset_1000dps_xyz);
					EEPROM.put(ADDRESS_EEPROM, calibration_eeprom);
					t_calibrateGyro = 0;
					updateLed(LED_PIN, 0); // turn off LED
					return true;
				}
				else {
					return false;
				}
			}
			else if ((rc_channelValue[THROTTLE] > 1950) && (rc_channelValue[YAW] < 1050)) {
				// hold right stick bottom center and left stick top-left to start accel calibration	(2s)
				t_calibrateGyro = 0;
				t_calibrateMag = 0;
				if (t_calibrateAccel == 0) {
					t_calibrateAccel = t_imuCalibration;
					return false;
				}
				else if ((t_imuCalibration - t_calibrateAccel) > 2000000) {
					updateLed(LED_PIN, 2); // turn on LED to indicate calibration
					imu.calibrate_accel(imuInterrupt, 5.0, 16, calibration_eeprom.accelOffset_32g_xyz);
					EEPROM.put(ADDRESS_EEPROM, calibration_eeprom);
					t_calibrateAccel = 0;
					updateLed(LED_PIN, 0); // turn off LED
					return true;
				}
				else {
					return false;
				}
			}
#ifdef USE_MAG
			else if ((rc_channelValue[THROTTLE] > 1950) && (rc_channelValue[YAW] > 1950)) {
				// hold right stick bottom center and left stick top-right to start mag calibration	(2s)
				t_calibrateGyro = 0;
				t_calibrateAccel = 0;
				if (t_calibrateMag == 0) {
					t_calibrateMag = t_imuCalibration;
					return false;
				}
				else if ((t_imuCalibration - t_calibrateMag) > 2000000) {
					updateLed(LED_PIN, 2); // turn on LED to indicate calibration
					imu.calibrate_mag(imuInterrupt, 60, 500, calibration_eeprom.magOffset_xyz, calibration_eeprom.magScale_xyz);
					EEPROM.put(ADDRESS_EEPROM, calibration_eeprom);
					t_calibrateMag = 0;
					updateLed(LED_PIN, 0); // turn off LED
					return true;
				}
				else {
					return false;
				}
			}
#endif
		}
	}
	
	t_calibrateGyro = 0;
	t_calibrateAccel = 0;
	t_calibrateMag = 0;
	
	return false;
}