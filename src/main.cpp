#include "main.h"
#include "common.h"
#include "calibration.h"
#include "motorsQuad.h"
#include "shapeTrajectory.h"
#include "kalmanFilter1D.h"
#include "sendSerial.h"

#include <iBus.h>
#include <ICM20948.h>
#include <MadgwickAHRS.h>
#include <ema_filter.h>
#include <PID_controller.h>
#include <BMP388_DEV.h>
#include <NMEAGPS.h>
#include <Plotter.h>

#include <Arduino.h>
#include <EEPROM.h>

// ! Motors are disabled when MOTORS_OFF is defined inside "common.h".
// ! Debug options can be enabled within "common.h"
// ! Parameters can be set inside "main.h".

#ifdef PLOT
Plotter p;
#endif

// radio control (rc)
IBUS rc;

// inertial measurement unit (imu)
ICM20948_SPI imu(IMU_CS_PIN, IMU_SPI_PORT);

// Madgwick filter for pose estimation
MADGWICK_AHRS madgwickFilter(BETA_INIT);

// barometer
BMP388_DEV barometer;

// 1D-Kalman-Filter combining barometer altitude and vertical acceleration into altitude
KalmanFilter1D altitudeFilter(TC_ALTITUDE_FILTER);

// gps parser
NMEAGPS gps;

// gps fix data
gps_fix fix;

// launch location determined before arming
NeoGPS::Location_t launch_location;

// location where rtl mode was entered
NeoGPS::Location_t rtl_location;

// motors
MotorsQuad motors(MOTOR_PIN_1, MOTOR_PIN_2, MOTOR_PIN_3, MOTOR_PIN_4, MOTOR_PWM_RESOLUTION, MOTOR_PWM_FREQUENCY);

// rate PID controller
PID_controller roll_rate_pid(P_ROLL_RATE, I_ROLL_RATE, D_ROLL_RATE, 0, 0, 250, 50, EMA_ROLL_RATE_P, EMA_ROLL_RATE_D);
PID_controller pitch_rate_pid(P_PITCH_RATE, I_PITCH_RATE, D_PITCH_RATE, 0, 0, 250, 50, EMA_PITCH_RATE_P, EMA_PITCH_RATE_D);
PID_controller yaw_rate_pid(P_YAW_RATE, I_YAW_RATE, D_YAW_RATE, 0, 0, 250, 100, EMA_YAW_RATE_P, EMA_YAW_RATE_D);

// vertical velocity PID controller for altitude hold
PID_controller velocity_v_pid(P_VELOCITY_V, I_VELOCITY_V, D_VELOCITY_V, 0, 0, THROTTLE_LIMIT - THROTTLE_HOVER, 200, EMA_VELOCITY_V_P, EMA_VELOCITY_V_D);

// horizontal velocity PID controllers for return to launch
PID_controller velocity_x_pid(P_VELOCITY_H, I_VELOCITY_H, D_VELOCITY_H, 0, 0, ROLL_PITCH_ANGLE_LIMIT, ROLL_PITCH_ANGLE_LIMIT * 0.2, EMA_VELOCITY_H_P, EMA_VELOCITY_H_D);
PID_controller velocity_y_pid(P_VELOCITY_H, I_VELOCITY_H, D_VELOCITY_H, 0, 0, ROLL_PITCH_ANGLE_LIMIT, ROLL_PITCH_ANGLE_LIMIT * 0.2, EMA_VELOCITY_H_P, EMA_VELOCITY_H_D);

// flight modes
enum class FlightMode { Stabilize, AltitudeHold, ReturnToLaunch } fMode;

// return to launch states
enum class RtlState { Climb, YawToLaunch, Return, YawToInitial, Descend } rtlState;

// calibration data
calibration_data calibration_eeprom;

// flag used to initialise the quadcopter pose and altitude after power on or calibration
bool initialised = false;

// initial quadcopter yaw (z-axis) angle
float yaw_angle_init;

// initial quadcopter altitude
float altitude_init;

// maximum quadcopter altitude
float altitude_max;

// Stores which errors occurred. Each bit belongs to a certain error.
// If the errorcode is unequal zero, arming is disabled.
// A power cycle is required to reset errors and enable arming.
uint8_t error_code;

// pointer on an array of 10 received rc channel values [1000; 2000]
uint16_t *rc_channelValue;

// throttle output
float throttle_out;

// flight setpoints
float roll_angle_sp, pitch_angle_sp;
float roll_rate_sp, pitch_rate_sp, yaw_rate_sp;
float altitude_sp;
float velocity_v_sp;
float velocity_x_sp, velocity_y_sp;

// manipulated variables
float roll_rate_mv, pitch_rate_mv, yaw_rate_mv;
float velocity_v_mv;

// accelerometer resolution
float accelRes;

// imu measurements
int16_t ax, ay, az;
float gx_rps, gy_rps, gz_rps;
int16_t mx, my, mz;

// pose
float roll_angle, pitch_angle, yaw_angle; // euler angles
float pose_q[4];						  // quaternion

// ned-acceleration relative to gravity
float a_n_rel, a_e_rel, a_d_rel;

// barometer altitude measurement
float baroAltitude;

// altitude
float altitude;

// vertical velocity
float velocity_v;

// velocities in ned-frame
float velocity_north, velocity_east;

// velocities in body-frame
float velocity_x, velocity_y; 

// position error in ned-frame
float north_position_error, east_position_error;

// position error in body-frame
float x_position_error, y_position_error; 

// heading
float heading;

// gps heading is only valid when moving (at roughly pedestrian speed)
bool heading_valid;

// bearing to launch location (clockwise from north)
float bearing;

// heading corrected yaw angle
float yaw_angle_corrected;

// yaw angle error
float yaw_angle_error;

// filtered gyro rates
float roll_rate, pitch_rate, yaw_rate;

// PID calculations are delayed until started flag becomes true, which happens when minimum throttle is reached
bool started = false;

// imu interrupt
volatile bool imuInterrupt = false;
void imuReady() {
	imuInterrupt = true;
}

// barometer interrupt
volatile bool barometerInterrupt = false;
void barometerReady() {
	barometerInterrupt = true;
}

void setup() {
	// setup built in LED
	pinMode(LED_PIN, OUTPUT);

	// turn off LED
	updateLED(LED_PIN, 0);

	// set default resolution for analog write, which is different from the one used for motor actuation
	analogWriteResolution(8);

#if defined(DEBUG) || defined(SEND_SERIAL)
	// initialise serial port for monitoring
	Serial.begin(115200);
	while (!Serial);
#endif

	// initialise serial port for rc (iBus) communication
	rcPort.begin(115200);
	while (!rcPort);

	// initialise serial port for gps
	gpsPort.begin(115200);
	while (!gpsPort);

	// initialise rc and return a pointer on the received rc channel values
	rc_channelValue = rc.begin(rcPort);

	// initialise SPI for imu communication
	IMU_SPI_PORT.begin();

	// setup imu interrupt pin
	pinMode(IMU_INTERRUPT_PIN, INPUT);

	// get calibration data from EEPROM
	EEPROM.get(ADDRESS_EEPROM, calibration_eeprom);

	// initialise imu
	if (!imu.init(calibration_eeprom.gyroOffset_1000dps_xyz, calibration_eeprom.accelOffset_32g_xyz, calibration_eeprom.magOffset_xyz, calibration_eeprom.magScale_xyz)) {
		// imu could not be initialised
		error_code |= ERROR_IMU; // set error value to disable arming
		DEBUG_PRINTLN(F("IMU error: Initialisation failed!"));
	}

	// read accelerometer resolution in g/bit
	imu.read_accelRes(accelRes);

#ifdef USE_BAR
	// initialise barometer with mode, pressure oversampling, temperature oversampling, IIR-Filter and standby time
	if (!barometer.begin(NORMAL_MODE, OVERSAMPLING_SKIP, OVERSAMPLING_SKIP, IIR_FILTER_32, TIME_STANDBY_5MS)) {
		// barometer could not be initialised
		error_code |= ERROR_BAR; // set error value to disable arming
		DEBUG_PRINTLN(F("BAR error: Initialisation failed!"));
	}

	// setup barometer interrupt pin
	pinMode(BAROMETER_INTERRUPT_PIN, INPUT);
	barometer.enableInterrupt();
#endif

	// attach interrupts
	attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), imuReady, FALLING);
#ifdef USE_BAR
	attachInterrupt(digitalPinToInterrupt(BAROMETER_INTERRUPT_PIN), barometerReady, RISING);
#endif

#ifdef PLOT
	// ! Start plotter
	p.Begin();

	// Add time graphs. Notice the effect of points displayed on the time scale
	//p.AddTimeGraph("throttle_out", 1000, "throttle_out", throttle_out);
	//p.AddTimeGraph("Angles", 1000, "r", roll_angle, "p", pitch_angle, "y", yaw_angle);
	//p.AddTimeGraph("Rates", 1000, "roll_rate", roll_rate, "pitch_rate", pitch_rate, "yaw_rate", yaw_rate);
	//p.AddTimeGraph("mv", 1000, "roll_rate_sp", roll_rate_sp, "pitch_rate_sp", pitch_rate_sp, "yaw_rate_sp", yaw_rate_sp);
	//p.AddTimeGraph("mv", 1000, "roll_rate_mv", roll_rate_mv, "pitch_rate_mv", pitch_rate_mv, "yaw_rate_mv", yaw_rate_mv);

	//p.AddTimeGraph("Relative ned-acceleration", 1000, "a_n_rel", a_n_rel, "a_e_rel", a_e_rel, "a_d_rel", a_d_rel);

	//p.AddTimeGraph("alt", 1000, "A", altitude, "bA", baroAltitude);
	//p.AddTimeGraph("alt", 1000, "A", altitude, "bA", baroAltitude/*, "aS", altitude_sp*/);
	//p.AddTimeGraph("v_vel", 1000, "V", velocity_v/*, "V_sp", velocity_v_sp*/);

	//p.AddTimeGraph("position_error_ne", 1000, "n_pe", north_position_error, "e_pe", east_position_error);
	//p.AddTimeGraph("position_error_xy", 1000, "x_pe", x_position_error, "y_pe", y_position_error);

	//p.AddTimeGraph("yawHeading", 1000, "yaw", yaw_angle, "heading", heading);

	//p.AddTimeGraph("bearing", 1000, "bearing", bearing);
	//p.AddTimeGraph("RTL", 1000, "yaw", yaw_angle, "heading", heading, "bearing", bearing, "yaw_angle_corrected", yaw_angle_corrected);
#endif
}

void loop() {
	static uint32_t t0, t; // variables to measure the imu update time
	static int32_t dt;     // measured imu update time in microseconds
	static float dt_s;     // measured imu update time in seconds

	// * update LED
	if ((error_code & !ERROR_GPS) != 0) {
		// blink LED very fast to indicate an error occurrence except gps
		updateLED(LED_PIN, 1, 200);
	}
	else if (motors.getState() == MotorsQuad::State::armed) {
		// blink LED normally to indicate armed status
		updateLED(LED_PIN, 1, 1000);
	}
	else if (!initialised || (error_code == ERROR_GPS)) {
		// blink LED fast to indicate quadcopter initialisation or gps search
		updateLED(LED_PIN, 1, 500);
	}
	else if (motors.getState() == MotorsQuad::State::disarmed) {
		// turn off LED to indicate disarmed status
		updateLED(LED_PIN, 0);
	}
	else {
		// turn on LED to indicate arming/disarming status
		updateLED(LED_PIN, 2);
	}

	// * update radio control (rc)
	rc.update();

	// * perform calibration, if motors are disarmed and rc calibration request was received
	if (calibration(motors, imu, rc_channelValue, calibration_eeprom)) {
		// reinitialise quadcopter, because calibration was performed
		initialised = false;
	}

	// * arm/disarm on rc command or disarm on failsafe conditions
	arm_failsafe(FS_CONFIG, dt);

	// update time
	t = micros();
	dt = (t - t0);			  // in us
	dt_s = (float)(dt)*1.e-6; // in s

	// * continue if imu interrupt has fired
	if (!imuInterrupt) {
		return;
	}
	imuInterrupt = false; // reset imu interrupt

	t0 = t;

	// * read accel and gyro measurements
	imu.read_accel_gyro_rps(ax, ay, az, gx_rps, gy_rps, gz_rps);

#ifdef USE_MAG
	// * read magnetometer
	static uint32_t dt_mag = 0;
	if (!(error_code & ERROR_MAG) && imu.read_mag(mx, my, mz)) {
		dt_mag = 0;
	}
	else if (initialised) {
		// check if magnetometer is still working
		dt_mag += dt;
		if ((dt_mag > SENSOR_DT_LIMIT) && !(error_code & ERROR_MAG)) {
			// Too much time has passed since the last magnetometer reading. Set error value, which will disable arming.
			error_code |= ERROR_MAG;

			mx = my = mz = 0; // error handling

			DEBUG_PRINTLN(F("Magnetometer error!"));
		}
	}
#endif

	// * calculate pose from sensor data using Madgwick filter
	madgwickFilter.get_euler_quaternion(dt_s, ax, ay, az, gx_rps, gy_rps, gz_rps, mx, my, mz, roll_angle, pitch_angle, yaw_angle, pose_q);

	// apply offset to z-axis pose in order to compensate for the sensor mounting orientation relative to the quadcopter frame
	yaw_angle += YAW_ANGLE_OFFSET;
#ifdef USE_GPS
	// this is necessary to match the gps heading
	yaw_angle = -yaw_angle;
#endif
	adjustAngleRange(0, 360, yaw_angle);

#ifdef USE_BAR
	// * get altitude from barometer
	static uint32_t dt_bar = 0;
	if (barometerInterrupt) {
		barometerInterrupt = false;
		dt_bar = 0;

		barometer.getAltitude(baroAltitude);
	}
	else if (initialised) {
		// check if barometer is still working
		dt_bar += dt;
		if ((dt_bar > SENSOR_DT_LIMIT) && !(error_code & ERROR_BAR)) {
			// Too much time has passed since the last barometer reading. Set error value, which will disable arming.
			error_code |= ERROR_BAR;

			DEBUG_PRINTLN(F("Barometer error!"));
		}
	}

	// ned-acceleration relative to gravity in m/s²
	accel_ned_rel(a_n_rel, a_e_rel, a_d_rel);

	// * calculate quadcopter altitude in m and vertical velocity in m/s
	altitudeFilter.update(a_d_rel, baroAltitude, dt_s);
	altitude = altitudeFilter.get_position();
	velocity_v = altitudeFilter.get_velocity();
#endif

#ifdef USE_GPS
	// * get gps location, speed and heading
	static uint32_t dt_gps = 0;
	if (gps.available(gpsPort)) {
		dt_gps = 0;

		fix = gps.read();

		// set error code in order to disable arming as long as there is no solid gps fix
		if (fix.valid.satellites && fix.satellites > 7 && fix.valid.hdop && fix.hdop < 6000 && fix.valid.location) {

			if (error_code & ERROR_GPS) {
				error_code &= !ERROR_GPS; // delete error value to enable arming and rtl

				DEBUG_PRINTLN(F("Solid GPS fix!"));
			}

			// update launch location if the quadcopter is disarmed
			if (motors.getState() == MotorsQuad::State::disarmed) {
				launch_location = fix.location;
			}
		}
		else if (!(error_code & ERROR_GPS)) {
			error_code |= ERROR_GPS; // set error value to disable arming and rtl

			DEBUG_PRINTLN(F("No solid GPS fix! Arming and RTL are disabled."));
		}

		if (fix.valid.location) {
			// get bearing to launch location (clockwise from north)
			bearing = fix.location.BearingToDegrees(launch_location);
		}

		if (fix.valid.velned) {
			// quadcopter north and east velocity
			velocity_north = fix.velocity_north * 0.01;
			velocity_east = fix.velocity_east * 0.01;
		}

		if (fix.valid.heading) {
			if (heading_valid == false) {
				heading_valid = true;

				// initialise filter with the current heading
				heading = fix.heading();
			}

			// filtered quadcopter heading
			heading = ema_filter(fix.heading(), heading, EMA_HEADING);
		}
		else {
			heading_valid = false;
		}
	}
	else {
		// check if gps is still working
		dt_gps += dt;
		if ((dt_gps > SENSOR_DT_LIMIT) && !(error_code & ERROR_GPS)) {
			// Too much time has passed since the last gps reading. Set error value, which will disable arming.
			error_code |= ERROR_GPS;

			// No valid heading. The heading filter will be reinitialised when it is valid again.
			heading_valid = false;

			DEBUG_PRINTLN(F("GPS error!"));
		}
	}
#endif

	// * initialise quadcopter after first run or calibration
	if (!initialised) {
		static uint8_t initStatus = 0;

		switch (initStatus) {
			case 0:
				t0 = micros();
				yaw_angle_init = -1000; // reset initial quadcopter z-axis angle
				altitude_init = -1000;  // reset initial quadcopter altitude
				disarmAndResetQuad();
				
				++initStatus;
				break;
			case 1:
				// estimate initial pose
				if (initPose(BETA_INIT, BETA, INIT_ANGLE_DIFFERENCE, INIT_RATE, dt_s)) {
					++initStatus;
				}
				break;

#ifdef USE_BAR
			case 2:
				// estimate initial altitude
				if (initAltitude(INIT_VELOCITY_V, dt_s)) {
					++initStatus;
				}
				break;
#endif

			default:
				// quadcopter initialisation successful
				initStatus = 0;
				initialised = true;
				break;
		}
		return;
	}

	roll_rate = gx_rps * RAD2DEG;
	pitch_rate = gy_rps * RAD2DEG;
	yaw_rate = gz_rps * RAD2DEG;

	// * calculate flight setpoints, manipulated variables and control motors, when armed
	if (motors.getState() == MotorsQuad::State::armed) {
		// set the maximum altitude for rtl
		if (fMode != FlightMode::ReturnToLaunch) {
			altitude_max = max(altitude, altitude_max);
		}

		// update flight mode
		switch (rc_channelValue[FMODE])
		{
			case 1000:
				fMode = FlightMode::Stabilize;
				break;

			default:
#ifdef USE_BAR
				// use AltitudeHold, but switch to Stabilize on barometer failure
				if (!(error_code & ERROR_BAR)) {
					fMode = FlightMode::AltitudeHold;
				} else {
					fMode = FlightMode::Stabilize;
				}
#else
				fMode = FlightMode::Stabilize;
#endif
				break;
		}

		// last flight mode
		static FlightMode fMode_last;

		static float throttle;
		// map throttle to [-1, 1]
		throttle = map((float) rc_channelValue[THROTTLE], 1000, 2000, -1, 1);
		// apply expo to throttle for less sensitivity around hover
		throttle = expo_curve(throttle, THROTTLE_EXPO);
		// map throttle through THROTTLE_ARMED, THROTTLE_HOVER and THROTTLE_LIMIT
		throttle = map3(throttle, -1, 0, 1, THROTTLE_ARMED, THROTTLE_HOVER, THROTTLE_LIMIT);

		// in order to ensure a smooth start, PID calculations are delayed until hover throttle is reached
		if (started) {
#if defined(USE_GPS) && defined(USE_BAR) && defined(USE_MAG)
			if ((rc_channelValue[RTL] == 2000) && !(error_code & (ERROR_GPS | ERROR_BAR))) {
				fMode = FlightMode::ReturnToLaunch;
			}
#endif

			if (fMode != fMode_last) {
				if ((fMode_last == FlightMode::AltitudeHold) || (fMode_last == FlightMode::ReturnToLaunch)) {
					velocity_v_sp = 0;
					velocity_v_pid.reset();
					velocity_v_mv = 0;

					if (fMode_last == FlightMode::ReturnToLaunch) {
						velocity_x_sp = 0;
						velocity_y_sp = 0;

						velocity_x_pid.reset();
						velocity_y_pid.reset();
					}
				}

				if (fMode == FlightMode::AltitudeHold) {
					altitude_sp = altitude;
				}
#ifdef USE_GPS
				else if (fMode == FlightMode::ReturnToLaunch) {
					if (fix.valid.location) {
						rtl_location = fix.location;
					}
					else {
						// cancel return to launch by setting gps error
						error_code |= ERROR_GPS;
					}

					// reset rtl state
					rtlState = RtlState::Climb;
				}
#endif
			}
			else {
				switch (fMode) {
					case FlightMode::Stabilize:
						throttle_out = throttle;
						break;

					case FlightMode::AltitudeHold:
						if (rc_channelValue[THROTTLE] < THROTTLE_DEADZONE_BOT) {
							// shape rc input to control downwards velocity
							velocity_v_sp = shape_velocity(map(rc_channelValue[THROTTLE], 1000, THROTTLE_DEADZONE_BOT, -VELOCITY_V_LIMIT, 0), ACCEL_V_MAX, velocity_v_sp, dt_s);

							altitude_sp = altitude;
						}
						else if (rc_channelValue[THROTTLE] > THROTTLE_DEADZONE_TOP) {
							// shape rc input to control upwards velocity
							velocity_v_sp = shape_velocity(map(rc_channelValue[THROTTLE], THROTTLE_DEADZONE_TOP, 2000, 0, VELOCITY_V_LIMIT), ACCEL_V_MAX, velocity_v_sp, dt_s);

							altitude_sp = altitude;
						}
						else {
							// shape vertical velocity to hold altitude
							velocity_v_sp = constrain(shape_position(altitude_sp - altitude, TC_ALTITUDE, ACCEL_V_MAX, velocity_v_sp, dt_s), -VELOCITY_V_LIMIT, VELOCITY_V_LIMIT);
						}

						// calculate manipulated variable for vertical velocity
						velocity_v_mv = velocity_v_pid.get_mv(velocity_v_sp, velocity_v, dt_s);

						// use throttle hover, so vertical velocity controller can take over smoothly
						throttle_out = THROTTLE_HOVER + velocity_v_mv;
						break;

#ifdef USE_GPS
					case FlightMode::ReturnToLaunch:
						// rtl state machine
						switch (rtlState) {
							case RtlState::Climb:
								// climb to the maximum altitude reached during flight with some added offset for safety
								altitude_sp = altitude; // TODO: Remove this after successful tests and uncomment altitude_sp below.
								//altitude_sp = altitude_max + RTL_RETURN_OFFSET;

								// keep the current heading
								yaw_angle_error = 0;

								// stay at the location where rtl was enabled (calculate position error (distance from rtl location) in north/east direction by holding latitude/longitude for a simple short distance approximation)
								north_position_error = rtl_location.DistanceRadians(NeoGPS::Location_t(fix.location.lat(), rtl_location.lon())) * NeoGPS::Location_t::EARTH_RADIUS_KM * 1000;
								east_position_error = rtl_location.DistanceRadians(NeoGPS::Location_t(rtl_location.lat(), fix.location.lon())) * NeoGPS::Location_t::EARTH_RADIUS_KM * 1000;

								// check if the altitude setpoint is reached
								if (abs(altitude - altitude_sp) < 2) {
									rtlState = RtlState::YawToLaunch;
								}
								break;

							case RtlState::YawToLaunch:
								// stay at the location where rtl was enabled (calculate position error (distance from rtl location) in north/east direction by holding latitude/longitude for a simple short distance approximation)
								north_position_error = rtl_location.DistanceRadians(NeoGPS::Location_t(fix.location.lat(), rtl_location.lon())) * NeoGPS::Location_t::EARTH_RADIUS_KM * 1000;
								east_position_error = rtl_location.DistanceRadians(NeoGPS::Location_t(rtl_location.lat(), fix.location.lon())) * NeoGPS::Location_t::EARTH_RADIUS_KM * 1000;

								// turn the quadcopter towards the launch location, but only if it is not too close
								if ((abs(north_position_error) > 4) || (abs(east_position_error) > 4)) {
									// turn the quadcopter towards the launch location
									yaw_angle_error = yaw_angle_corrected - bearing;

									// check if the quadcopter is heading towards the launch location
									if (abs(yaw_angle_error) < 5) {
										rtlState = RtlState::Return;
									}
								}
								else {
									rtlState = RtlState::Return;
								}
								break;

							case RtlState::Return:
								// return to the launch location (calculate position error (distance from launch) in north/east direction by holding latitude/longitude for a simple short distance approximation)
								north_position_error = launch_location.DistanceRadians(NeoGPS::Location_t(fix.location.lat(), launch_location.lon())) * NeoGPS::Location_t::EARTH_RADIUS_KM * 1000;
								east_position_error = launch_location.DistanceRadians(NeoGPS::Location_t(launch_location.lat(), fix.location.lon())) * NeoGPS::Location_t::EARTH_RADIUS_KM * 1000;

								// turn the quadcopter towards the launch location, but only if it is not too close
								if ((abs(north_position_error) > 4) || (abs(east_position_error) > 4)) {
									// turn the quadcopter towards the launch location
									yaw_angle_error = yaw_angle_corrected - bearing;
								}

								// check if the launch location is reached
								if ((abs(north_position_error) < 2) && (abs(east_position_error) < 2)) {
									rtlState = RtlState::YawToInitial;
								}
								break;

							case RtlState::YawToInitial:
								// stay at the launch location (calculate position error (distance from launch location) in north/east direction by holding latitude/longitude for a simple short distance approximation)
								north_position_error = launch_location.DistanceRadians(NeoGPS::Location_t(fix.location.lat(), launch_location.lon())) * NeoGPS::Location_t::EARTH_RADIUS_KM * 1000;
								east_position_error = launch_location.DistanceRadians(NeoGPS::Location_t(launch_location.lat(), fix.location.lon())) * NeoGPS::Location_t::EARTH_RADIUS_KM * 1000;

								// turn the quadcopter to the initial yaw angle determined when arming
								yaw_angle_error = yaw_angle_corrected - yaw_angle_init;

								// check if the initial yaw angle is reached
								if (abs(yaw_angle_error) < 5) {
									rtlState = RtlState::Descend;
								}
								break;

							case RtlState::Descend:
								// stay at the launch location (calculate position error (distance from launch location) in north/east direction by holding latitude/longitude for a simple short distance approximation)
								north_position_error = launch_location.DistanceRadians(NeoGPS::Location_t(fix.location.lat(), launch_location.lon())) * NeoGPS::Location_t::EARTH_RADIUS_KM * 1000;
								east_position_error = launch_location.DistanceRadians(NeoGPS::Location_t(launch_location.lat(), fix.location.lon())) * NeoGPS::Location_t::EARTH_RADIUS_KM * 1000;

								// keep the initial yaw angle determined when arming
								yaw_angle_error = yaw_angle_corrected - yaw_angle_init;

								// descend to the initial altitude after arming with some added offset
								//altitude_sp = altitude_init + RTL_DESCEND_OFFSET;
								break;

							default:
								break;
						}

						// adjust the yaw angle error range to [-180, 180) in order to make sure the quadcopter turns the shortest way
						adjustAngleRange(-180, 180, yaw_angle_error);

						// shape vertical velocity
						velocity_v_sp = constrain(shape_position(altitude_sp - altitude, TC_ALTITUDE, ACCEL_V_MAX, velocity_v_sp, dt_s), -VELOCITY_V_LIMIT, VELOCITY_V_LIMIT);

						// calculate manipulated variable for vertical velocity
						velocity_v_mv = velocity_v_pid.get_mv(velocity_v_sp, velocity_v, dt_s);

						// use throttle hover, so vertical velocity controller can take over smoothly
						throttle_out = THROTTLE_HOVER + velocity_v_mv;
						break;
#endif
					default:
						break;
				}
			}

			// Tilt compensated thrust: Increase throttle when the quadcopter is tilted, to compensate for height loss during horizontal movement.
			// Note: In order to maintain stability, throttle is limited to the throttle limit.
			throttle_out = constrain((float)(throttle_out - 1000) / (pose_q[0] * pose_q[0] - pose_q[1] * pose_q[1] - pose_q[2] * pose_q[2] + pose_q[3] * pose_q[3]) + 1000, 1000, THROTTLE_LIMIT);

			if (fMode == FlightMode::ReturnToLaunch) {
#ifdef USE_GPS
				// Calculate heading corrected yaw angle.
				// Note: The correction can only make sense if the movement necessary to determine the heading is not caused by wind, so the quadcopter needs to be tilted.
				static float correction;
				if (heading_valid && ((abs(roll_angle) > 5) || (abs(pitch_angle) > 5))) {
					correction = bearing - heading; // TODO: This probably needs some major EMA filtering, so the yaw angle is only corrected very very slowly, but try to get it running without correction first.
					correction = 0;
				}
				yaw_angle_corrected = yaw_angle + correction;
				adjustAngleRange(0, 360, yaw_angle_corrected);

				// transform the distance from ned- to body-frame
				x_position_error = north_position_error * cos(yaw_angle_corrected * DEG2RAD) + east_position_error * sin(yaw_angle_corrected * DEG2RAD);
				y_position_error = north_position_error * (-sin(yaw_angle_corrected * DEG2RAD)) + east_position_error * cos(yaw_angle_corrected * DEG2RAD); 

				// shape x- and y-axis velocities
				velocity_x_sp = constrain(shape_position(x_position_error, TC_DISTANCE, ACCEL_H_MAX, velocity_x_sp, dt_s), -VELOCITY_XY_LIMIT, VELOCITY_XY_LIMIT);
				velocity_y_sp = constrain(shape_position(y_position_error, TC_DISTANCE, ACCEL_H_MAX, velocity_y_sp, dt_s), -VELOCITY_XY_LIMIT, VELOCITY_XY_LIMIT);

				// transform the velocity from ned- to body-frame.
				velocity_x = velocity_north * cos(yaw_angle_corrected * DEG2RAD) + velocity_east * sin(yaw_angle_corrected * DEG2RAD);
				velocity_y = velocity_north * (-sin(yaw_angle_corrected * DEG2RAD)) + velocity_east * cos(yaw_angle_corrected * DEG2RAD); 

				// calculate angle setpoints from velocity error for angle PID controller (cascade controller)
				pitch_angle_sp = velocity_x_pid.get_mv(velocity_x_sp, velocity_x, dt_s);
				roll_angle_sp = velocity_y_pid.get_mv(velocity_y_sp, velocity_y, dt_s);

				// yaw rate setpoint
				yaw_rate_sp = shape_position(yaw_angle_error, TC_YAW_ANGLE, ACCEL_MAX_YAW, yaw_rate_sp, dt_s);
#endif
			}
			else {
				// roll and pitch angle setpoints
				roll_angle_sp = map((float)rc_channelValue[ROLL], 1000, 2000, -ROLL_PITCH_ANGLE_LIMIT, ROLL_PITCH_ANGLE_LIMIT);
				pitch_angle_sp = map((float)rc_channelValue[PITCH], 1000, 2000, -ROLL_PITCH_ANGLE_LIMIT, ROLL_PITCH_ANGLE_LIMIT);

				// yaw rate setpoint
				if (rc_channelValue[THROTTLE] < 1050) {
					// if throttle is too low, disable setting a yaw rate, since it might cause problems when disarming
					yaw_rate_sp = shape_velocity(0, ACCEL_MAX_YAW, yaw_rate_sp, dt_s);
				} else {
					yaw_rate_sp = shape_velocity(map((float)rc_channelValue[YAW], 1000, 2000, YAW_RATE_LIMIT, -YAW_RATE_LIMIT), ACCEL_MAX_YAW, yaw_rate_sp, dt_s);
				}
			}

			// roll and pitch rate setpoints
			roll_rate_sp = shape_position(roll_angle_sp - roll_angle, TC_ROLL_PITCH_ANGLE, ACCEL_MAX_ROLL_PITCH, roll_rate_sp, dt_s);
			pitch_rate_sp = shape_position(pitch_angle_sp - pitch_angle, TC_ROLL_PITCH_ANGLE, ACCEL_MAX_ROLL_PITCH, pitch_rate_sp, dt_s);

			// TODO: Remove this test code
			static float p_rate, i_rate, d_rate;

			/*p_rate = constrain(map((float) rc_channelValue[4], 1000, 2000, 1.75, 2.5), 1.75, 2.5);
			//i_rate = constrain(map((float) rc_channelValue[5], 1000, 2000, 0.75, 1.75), 0.75, 1.75);
			d_rate = constrain(map((float) rc_channelValue[5], 1000, 2000, 0.02, 0.03), 0.02, 0.03);

			roll_rate_pid.set_K_p(p_rate);
			//roll_rate_pid.set_K_i(i_rate);
			roll_rate_pid.set_K_d(d_rate);

			pitch_rate_pid.set_K_p(p_rate);
			//pitch_rate_pid.set_K_i(i_rate);
			pitch_rate_pid.set_K_d(d_rate);*/

			/*p_rate = constrain(map((float) rc_channelValue[4], 1000, 2000, 3.5, 4.5), 3.5, 4.5);
			i_rate = constrain(map((float) rc_channelValue[5], 1000, 2000, 2.0, 3.0), 2.0, 3.0);
			//d_rate = constrain(map((float) rc_channelValue[5], 1000, 2000, -0.001, 0.01), 0, 0.01);

			yaw_rate_pid.set_K_p(p_rate);
			yaw_rate_pid.set_K_i(i_rate);
			//yaw_rate_pid.set_K_d(0);*/

			// TODO: Tune PID-controller for vertical velocity.
			p_rate = constrain(map((float)rc_channelValue[4], 1000, 2000, 250, 500), 250, 500);
			i_rate = constrain(map((float)rc_channelValue[5], 1000, 2000, 5, 10), 5, 100);
			//d_rate = constrain(map((float) rc_channelValue[5], 1000, 2000, -0.001, 0.01), 0, 0.01);

			velocity_v_pid.set_K_p(p_rate);
			velocity_v_pid.set_K_i(i_rate);
			//yaw_rate_pid.set_K_d(0);*/

			// calculate manipulated variables for rates
			roll_rate_mv = roll_rate_pid.get_mv(roll_rate_sp, roll_rate, dt_s);
			pitch_rate_mv = pitch_rate_pid.get_mv(pitch_rate_sp, pitch_rate, dt_s);
			yaw_rate_mv = yaw_rate_pid.get_mv(yaw_rate_sp, yaw_rate, dt_s);
		}
		else {
			altitude_sp = altitude;

			throttle_out = throttle;
			if (throttle_out > THROTTLE_HOVER) {
				started = true;
				DEBUG_PRINTLN(F("Started!"));
			}
		}

		fMode_last = fMode;
	}
	else if (motors.getState() == MotorsQuad::State::disarmed) {
		// store the initial yaw angle and altitude before the quadcopter gets armed
		yaw_angle_init = yaw_angle;
		altitude_init = altitude;

		// reset the maximum altitude
		altitude_max = -10000;
	}

	// motor mixing
	motors.output(
		constrain(throttle_out + roll_rate_mv - pitch_rate_mv + yaw_rate_mv, 1000, 2000),
		constrain(throttle_out - roll_rate_mv - pitch_rate_mv - yaw_rate_mv, 1000, 2000),
		constrain(throttle_out - roll_rate_mv + pitch_rate_mv + yaw_rate_mv, 1000, 2000),
		constrain(throttle_out + roll_rate_mv + pitch_rate_mv - yaw_rate_mv, 1000, 2000));

#if defined(DEBUG) || defined(SEND_SERIAL) || defined(PLOT)
	// run serial print at a rate independent of the main loop (micros() - t0_serial = 16666 for 60 Hz update rate)
	static uint32_t t0_serial = micros();
#endif

#ifdef DEBUG
	if (micros() - t0_serial > 16666) {
		t0_serial = micros();

		//DEBUG_PRINT(ax); DEBUG_PRINT("\t"); DEBUG_PRINT(ay); DEBUG_PRINT("\t"); DEBUG_PRINTLN(az);
		//DEBUG_PRINT(gx_rps); DEBUG_PRINT("\t"); DEBUG_PRINT(gy_rps); DEBUG_PRINT("\t"); DEBUG_PRINTLN(gz_rps);
		//DEBUG_PRINT(mx); DEBUG_PRINT("\t"); DEBUG_PRINT(my); DEBUG_PRINT("\t"); DEBUG_PRINTLN(mz);

		//static float roll_angle_accel, pitch_angle_accel;
		//accelAngles(roll_angle_accel, pitch_angle_accel);
		//DEBUG_PRINT(roll_angle_accel); DEBUG_PRINT("\t"); DEBUG_PRINTLN(pitch_angle_accel);

		//DEBUG_PRINT(map((float) rc_channelValue[ROLL], 1000, 2000, -ROLL_PITCH_ANGLE_LIMIT, ROLL_PITCH_ANGLE_LIMIT)); DEBUG_PRINT("\t"); DEBUG_PRINTLN(roll_angle_sp);
		//DEBUG_PRINT(map((float) rc_channelValue[YAW], 1000, 2000, -YAW_RATE_LIMIT, YAW_RATE_LIMIT)); DEBUG_PRINT("\t"); DEBUG_PRINTLN(yaw_rate_sp);
		//DEBUG_PRINT(map((float) rc_channelValue[THROTTLE], 1000, 2000, 1000, THROTTLE_LIMIT)); DEBUG_PRINT("\t"); DEBUG_PRINTLN(throttle_out);
		//DEBUG_PRINT(map((float) (rc_channelValue[THROTTLE] - 1000) / (cos(roll_angle * DEG2RAD) * cos(pitch_angle * DEG2RAD)) + 1000, 1000, 2000, 1000, THROTTLE_LIMIT)); DEBUG_PRINT("\t"); DEBUG_PRINTLN(throttle_out);

		//DEBUG_PRINTLN(roll_rate_sp);

		//DEBUG_PRINT(roll_angle); DEBUG_PRINT("\t"); DEBUG_PRINT(pitch_angle); DEBUG_PRINT("\t"); DEBUG_PRINT(yaw_angle); DEBUG_PRINT("\t");
		//DEBUG_PRINTLN(yaw_rate_sp);
		//DEBUG_PRINT(roll_rate); DEBUG_PRINT("\t"); DEBUG_PRINT(pitch_rate); DEBUG_PRINT("\t"); DEBUG_PRINTLN(yaw_rate);

		//DEBUG_PRINT(yaw_angle); DEBUG_PRINT("\t"); DEBUG_PRINTLN(heading);

		//DEBUG_PRINTLN(dt);
		//DEBUG_PRINTLN();

		// print channel values
		/*for (int i=0; i<10 ; i++) {
			DEBUG_PRINT(rc_channelValue[i]); DEBUG_PRINT("\t");
		}
		DEBUG_PRINTLN();*/
	}
#elif defined(SEND_SERIAL)
	if (micros() - t0_serial > 16666) {
		t0_serial = micros();
		// Visualize data with "Processing"
		sendSerial(dt, roll_angle, pitch_angle, yaw_angle);
	}
#elif defined(PLOT)
	// TODO: Check if this update rate is still too fast
	if (micros() - t0_serial > 25000) {
		t0_serial = micros();
		// Plot data with "Processing"
		p.Plot();
	}
#endif
}

// estimate initial pose
bool initPose(float beta_init, float beta, float init_angleDifference, float init_rate, float dt_s) {
	static state initPose_state = state::init;

	switch (initPose_state) {
		case state::init:
			DEBUG_PRINTLN(F("Estimating initial pose. Keep device at rest ..."));

			// set higher beta value to speed up pose estimation
			madgwickFilter.set_beta(beta_init);

			initPose_state = state::busy;
			break;

		case state::busy:
			// angles calculated from accelerometer
			float roll_angle_accel, pitch_angle_accel;
			accelAngles(roll_angle_accel, pitch_angle_accel);

			// initial pose is estimated if filtered x- and y-axis angles, as well as z-axis angular velocity, converged
			if ((abs(roll_angle_accel - roll_angle) < init_angleDifference) && (abs(pitch_angle_accel - pitch_angle) < init_angleDifference)
			&& ((abs(yaw_angle - yaw_angle_init) / dt_s) < init_rate)) {
				// reduce beta value, since filtered angles have stabilized during initialisation
				madgwickFilter.set_beta(beta);

				initPose_state = state::init;

				DEBUG_PRINTLN(F("Initial pose estimated."));
				DEBUG_PRINTLN2(abs(roll_angle_accel - roll_angle), 6);
				DEBUG_PRINTLN2(abs(pitch_angle_accel - pitch_angle), 6);
				DEBUG_PRINTLN2(abs(yaw_angle - yaw_angle_init) / dt_s, 6);

				return true;
			}
			yaw_angle_init = yaw_angle;

#ifdef DEBUG
			// run serial print at a rate independent of the main loop
			static float t_serial_s = 0;
			if (t_serial_s > 1e-1) {
				DEBUG_PRINT(roll_angle_accel); DEBUG_PRINT("\t"); DEBUG_PRINTLN(pitch_angle_accel);
				DEBUG_PRINT(roll_angle); DEBUG_PRINT("\t"); DEBUG_PRINT(pitch_angle); DEBUG_PRINT("\t"); DEBUG_PRINTLN(yaw_angle);

				t_serial_s = 0;
			}
			t_serial_s += dt_s;
#endif
			break;

		default:
			break;
	}

	return false;
}

// calculate accelerometer x and y angles in degrees
void accelAngles(float &roll_angle_accel, float &pitch_angle_accel) {
	roll_angle_accel = atan2(ay, az) * RAD2DEG;
	pitch_angle_accel = atan2(-ax, sqrt(pow(ay, 2) + pow(az, 2))) * RAD2DEG;
}

// estimate initial altitude
bool initAltitude(float init_velocity_v, float dt_s) {
	static state initAltitude_state = state::init;

	switch (initAltitude_state) {
		case state::init:
			DEBUG_PRINTLN(F("Estimating initial altitude. Keep device at rest ..."));

			initAltitude_state = state::busy;
			break;

		case state::busy:
			static float dt_altitude_s = 0;

			if (dt_altitude_s > 1) {
				// initial altitude is estimated if vertical velocity converged
				// TODO: Use velocity from altitudeFilter instead
				if ((abs(altitude - altitude_init) / dt_altitude_s) < init_velocity_v) {
					initAltitude_state = state::init;

					DEBUG_PRINTLN(F("Initial altitude estimated."));
					DEBUG_PRINTLN2(abs(altitude - altitude_init) / dt_altitude_s, 2);

					return true;
				}
				altitude_init = altitude;
				dt_altitude_s = 0;

				DEBUG_PRINTLN(altitude);
			}
			dt_altitude_s += dt_s;

			break;

		default:
			break;
	}

	return false;
}

// calculate the acceleration in ned-frame relative to gravity
void accel_ned_rel(float &a_n_relative, float &a_e_relative, float &a_d_relative) {
	// acceleration of gravity is m/s²
	static const float G = 9.81;

	// acceleration quaternion in body-frame
	static float a_q[4];
	a_q[0] = 0; a_q[1] = ax; a_q[2] = ay; a_q[3] = az;

	// acceleration quaternion in ned-frame
	static float a_ned_q[4];
	body2nedFrame(pose_q, a_q, a_ned_q);

	// get ned-acceleration relative to gravity in m/s²
	a_n_relative = a_ned_q[1] * accelRes * G;
	a_e_relative = a_ned_q[2] * accelRes * G;
	a_d_relative = (a_ned_q[3] * accelRes - 1) * G;
}

// arm/disarm on rc command or disarm on failsafe conditions
void arm_failsafe(uint8_t fs_config, int32_t dt) {
	static uint32_t t_arm_failsafe;
	t_arm_failsafe = micros();

	// -------------------- automatically disarm when no start happens within 15 seconds
	static uint32_t t_auto_disarm = 0;
	if (!started && (motors.getState() == MotorsQuad::State::armed)) {
		if (t_auto_disarm == 0) {
			t_auto_disarm = t_arm_failsafe;
		}
		else if ((t_arm_failsafe - t_auto_disarm) > 15000000) {
			disarmAndResetQuad();
			t_auto_disarm = 0;
		}
	}
	else {
		t_auto_disarm = 0;
	}

	// -------------------- arm and disarm on rc command
	static uint32_t t_arm = 0, t_disarm = 0;
	if (rc_channelValue[ARM] == 2000 && initialised) { // arm switch needs to be set and quadcopter needs to be initialised to enable arming
		if ((rc_channelValue[THROTTLE] < 1050) && (((rc_channelValue[ROLL] > 1450) && (rc_channelValue[ROLL] < 1550)) && ((rc_channelValue[PITCH] > 1450) && (rc_channelValue[PITCH] < 1550)))) {
			if ((rc_channelValue[YAW] > 1950) && (motors.getState() == MotorsQuad::State::disarmed)) {
				// hold left stick bottom-right and keep right stick centered (3s) to complete arming
				t_disarm = 0;
				if (t_arm == 0) {
					t_arm = t_arm_failsafe;
				}
				else if ((t_arm_failsafe - t_arm) > 3000000) {
					motors.arm();
					t_arm = 0;
				}
			}
			else if ((rc_channelValue[YAW] < 1050) && ((motors.getState() == MotorsQuad::State::armed) || (motors.getState() == MotorsQuad::State::arming))) {
				// hold left stick bottom-left and keep right stick centered (3s) to complete disarming
				t_arm = 0;
				if (t_disarm == 0) {
					t_disarm = t_arm_failsafe;
				}
				else if ((t_arm_failsafe - t_disarm) > 3000000) {
					disarmAndResetQuad();
					t_disarm = 0;
				}
			}
			else {
				t_arm = 0;
				t_disarm = 0;
			}
		}
		else {
			t_arm = 0;
			t_disarm = 0;
		}
	}
	else if ((motors.getState() == MotorsQuad::State::armed) || (motors.getState() == MotorsQuad::State::arming)) {
		disarmAndResetQuad();
		t_arm = 0;
		t_disarm = 0;
	}
	else {
		t_arm = 0;
		t_disarm = 0;
	}

	// -------------------- disarm on failsafe conditions
	if ((motors.getState() == MotorsQuad::State::armed) || (motors.getState() == MotorsQuad::State::arming)) {
		// imu failsafe
		if ((FS_CONFIG & FS_IMU) == FS_IMU) {
			if (dt > FS_IMU_DT_LIMIT) {
				// limit for imu update time exceeded
				disarmAndResetQuad();
				error_code |= ERROR_IMU;
				DEBUG_PRINTLN(F("IMU failsafe caused by major IMU error!"));
			}
		}

		// quadcopter motion failsafe
		static uint32_t t_fs_motion = 0;
		if ((FS_CONFIG & FS_MOTION) == FS_MOTION) {
			if ((abs(roll_angle) > FS_MOTION_ANGLE_LIMIT) || (abs(pitch_angle) > FS_MOTION_ANGLE_LIMIT) || (abs(yaw_rate) > FS_MOTION_RATE_LIMIT)) {
				// angle or angular rate limit exceeded
				if (t_fs_motion == 0) {
					t_fs_motion = t_arm_failsafe;
				}
				else if ((t_arm_failsafe - t_fs_motion) > FS_MOTION_CONTROL_DT_LIMIT) {
					disarmAndResetQuad();
					t_fs_motion = 0;
					DEBUG_PRINTLN(F("Motion failsafe!"));
				}
			}
			else {
				t_fs_motion = 0;
			}
		}
		else {
			t_fs_motion = 0;
		}

		// failsafe conditions after the quadcopter started
		if (started) {
			// quadcopter control failsafe
			static uint32_t t_fs_control = 0;
			if ((FS_CONFIG & FS_CONTROL) == FS_CONTROL) {
				if ((abs(roll_angle_sp - roll_angle) > FS_CONTROL_ANGLE_DIFF) || (abs(pitch_angle_sp - pitch_angle) > FS_CONTROL_ANGLE_DIFF) || (abs(yaw_rate_sp - yaw_rate) > FS_CONTROL_RATE_DIFF)) {
					// difference to control values for angle and angular rate exceeded
					if (t_fs_control == 0) {
						t_fs_control = t_arm_failsafe;
					}
					else if ((t_arm_failsafe - t_fs_control) > FS_MOTION_CONTROL_DT_LIMIT) {
						disarmAndResetQuad();
						t_fs_control = 0;
						DEBUG_PRINTLN(F("Control failsafe!"));
					}
				}
				else {
					t_fs_control = 0;
				}
			}
			else {
				t_fs_control = 0;
			}
		}
	}
}

// disarm and reset quadcopter
void disarmAndResetQuad() {
	// set flight setpoints to zero
	roll_angle_sp = 0;
	pitch_angle_sp = 0;

	roll_rate_sp = 0;
	pitch_rate_sp = 0;
	yaw_rate_sp = 0;

	throttle_out = 1000;
	velocity_v_sp = 0;

	altitude_sp = altitude;

	velocity_x_sp = 0;
	velocity_y_sp = 0;

	// reset PID controller
	roll_rate_pid.reset();
	pitch_rate_pid.reset();
	yaw_rate_pid.reset();
	velocity_v_pid.reset();
	velocity_x_pid.reset();
	velocity_y_pid.reset();

	// reset altitude filter
	altitudeFilter.reset();

	roll_rate_mv = 0;
	pitch_rate_mv = 0;
	yaw_rate_mv = 0;
	velocity_v_mv = 0;

	// disarm motors
	motors.disarm();

	// set started state to false - minimum throttle is required again to start the motors and PID calculations
	started = false;
}
