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
// ! Debug options can be enabled within "common.h".
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

// 1D-Kalman-Filter combining raw barometer altitude and vertical acceleration
KalmanFilter1D altitudeFilter(TC_ALTITUDE_FILTER);

// gps parser
NMEAGPS gps;

// gps fix data
gps_fix fix;

// current location
NeoGPS::Location_t current_location;

// launch location determined before arming
NeoGPS::Location_t launch_location;

// location where rtl mode was entered
NeoGPS::Location_t rtl_location;

// motors
MotorsQuad motors(MOTOR_PIN_1, MOTOR_PIN_2, MOTOR_PIN_3, MOTOR_PIN_4, MOTOR_PWM_RESOLUTION, MOTOR_PWM_FREQUENCY);

// rate PID controller
PID_controller roll_rate_pid(P_ROLL_RATE, I_ROLL_RATE, D_ROLL_RATE, 250, 75, EMA_ROLL_RATE_P, EMA_ROLL_RATE_D, true);
PID_controller pitch_rate_pid(P_PITCH_RATE, I_PITCH_RATE, D_PITCH_RATE, 250, 75, EMA_PITCH_RATE_P, EMA_PITCH_RATE_D, true);
PID_controller yaw_rate_pid(P_YAW_RATE, I_YAW_RATE, D_YAW_RATE, 250, 100, EMA_YAW_RATE_P, EMA_YAW_RATE_D, true);

// vertical velocity PID controller for altitude hold
PID_controller velocity_v_pid(P_VELOCITY_V, I_VELOCITY_V, D_VELOCITY_V, THROTTLE_LIMIT - THROTTLE_HOVER, 200, EMA_VELOCITY_V_P, EMA_VELOCITY_V_D);

// horizontal velocity PID controllers for return to launch
PID_controller velocity_x_pid(P_VELOCITY_H, I_VELOCITY_H, D_VELOCITY_H, ROLL_PITCH_ANGLE_LIMIT, ROLL_PITCH_ANGLE_LIMIT * 0.2, EMA_VELOCITY_H_P, EMA_VELOCITY_H_D);
PID_controller velocity_y_pid(P_VELOCITY_H, I_VELOCITY_H, D_VELOCITY_H, ROLL_PITCH_ANGLE_LIMIT, ROLL_PITCH_ANGLE_LIMIT * 0.2, EMA_VELOCITY_H_P, EMA_VELOCITY_H_D);

// flight modes
enum class FlightMode { Stabilize, AltitudeHold, ReturnToLaunch } fMode;

// return to launch states
enum class RtlState { Init, Wait, Climb, YawToLaunch, Return, YawToInitial, Descend } rtlState;

// calibration data
calibration_data calibration_eeprom;

// flag used to initialise the quadcopter pose and altitude after power on or calibration
bool initialised = false;

// initial quadcopter yaw (z-axis) angle
float yaw_angle_init;

// initial quadcopter barometer altitude
float baroAltitude_init;

// maximum quadcopter altitude relative to starting ground
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

// ema filtered acceleration
float ax_filtered, ay_filtered, az_filtered;

// pose
float roll_angle, pitch_angle, yaw_angle, yaw_angle_rad; // euler angles
float pose_q[4]; // quaternion

//float roll_angle_accel, pitch_angle_accel;

// ned-acceleration relative to gravity
float a_n_rel, a_e_rel, a_d_rel;

// raw barometer altitude
float baroAltitudeRaw;

// filtered raw barometer altitude using altitudeFilter
float baroAltitude;

// altitude relative to starting ground
float altitude;

// vertical velocity
float velocity_v;

// velocities in ned-frame
float velocity_north, velocity_east;

// velocities in horizontal frame
float velocity_x, velocity_y; 

// distance to target location
float distance;

// distance to target location in horizontal frame 
float distance_x, distance_y;

// bearing to target location (clockwise from north)
float bearing, bearing_rad;

// quadcopter heading
float heading;

// heading correction to compensate inaccurate horizontal movement caused by bad compass measurements and wind 
float headingCorrection, headingCorrection_rad;

// distance to target yaw angle
float distance_yaw;

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
	//p.AddTimeGraph("alt", 1000, "A", baroAltitude, "bA", baroAltitudeRaw);
	//p.AddTimeGraph("v_vel", 1000, "V", velocity_v/*, "V_sp", velocity_v_sp*/);

	//p.AddTimeGraph("bearing", 1000, "bearing", bearing);
	//p.AddTimeGraph("distance_xy", 1000, "d_x", distance_x, "d_y", distance_y);
	//p.AddTimeGraph("velocity", 1000, "v_n", velocity_north, "v_e", velocity_east, "v_x", velocity_x, "v_y", velocity_y);

	//p.AddTimeGraph("yawTune", 1000, "yaw_rate_sp", yaw_rate_sp, "yaw_rate", yaw_rate, "yaw_angle", yaw_angle);
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
	dt = (t - t0);					// in us
	dt_s = (float) (dt) * 1.e-6;	// in s

	// * continue if imu interrupt has fired
	if (!imuInterrupt) {
		return;
	}
	imuInterrupt = false; // reset imu interrupt

	t0 = t;

	// * read accel and gyro measurements
	imu.read_accel_gyro_rps(ax, ay, az, gx_rps, gy_rps, gz_rps);

	// filter accelerometer measurements
	ax_filtered = ema_filter(ax, ax_filtered, EMA_ACCEL);
	ay_filtered = ema_filter(ay, ay_filtered, EMA_ACCEL);
	az_filtered = ema_filter(az, az_filtered, EMA_ACCEL);

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

	//accelAngles(roll_angle_accel, pitch_angle_accel);

	// * calculate pose from sensor data using Madgwick filter
	madgwickFilter.get_euler_quaternion(dt_s, ax_filtered, ay_filtered, az_filtered, gx_rps, gy_rps, gz_rps, mx, my, mz, roll_angle, pitch_angle, yaw_angle, pose_q);

	// apply offset to z-axis pose in order to compensate for the sensor mounting orientation relative to the quadcopter frame
	yaw_angle += YAW_ANGLE_OFFSET;
#ifdef USE_GPS
	// match yaw angle and gps heading orientation
	yaw_angle = -yaw_angle;
#endif
	adjustAngleRange(0, 360, yaw_angle);
	yaw_angle_rad = yaw_angle * RAD_PER_DEG;

#ifdef USE_BAR
	// * get raw altitude from barometer
	static uint32_t dt_bar = 0;
	if (barometerInterrupt) {
		barometerInterrupt = false;
		dt_bar = 0;

		barometer.getAltitude(baroAltitudeRaw);
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
	altitudeFilter.update(a_d_rel, baroAltitudeRaw, dt_s);
	baroAltitude = altitudeFilter.get_position();
	velocity_v = altitudeFilter.get_velocity();

	// altitude relative to starting ground
	altitude = baroAltitude - baroAltitude_init;
#endif

#ifdef USE_GPS
	// * get gps data
	static uint32_t dt_gps = 0;
	if (gps.available(gpsPort)) {
		dt_gps = 0;

		fix = gps.read();

		// set error code in order to disable arming as long as there is no solid gps fix
		if (fix.valid.satellites && fix.satellites > 5 && fix.valid.hdop && fix.hdop < 7000 && fix.valid.location) {
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
			current_location = fix.location;
		}

		if (fix.valid.heading) {
			heading = fix.heading();

			if (fix.valid.speed) {
				// calculate north and east velocity
				fix.calculateNorthAndEastVelocityFromSpeedAndHeading();

				velocity_north = (float) fix.velocity_north * 0.01;
				velocity_east = (float) fix.velocity_east * 0.01;
			}
		}
	}
	else {
		// check if gps is still working
		dt_gps += dt;
		if ((dt_gps > SENSOR_DT_LIMIT) && !(error_code & ERROR_GPS)) {
			// Too much time has passed since the last gps reading. Set error value, which will disable arming.
			error_code |= ERROR_GPS;

			DEBUG_PRINTLN(F("GPS error!"));
		}
	}
#endif

	// * initialise quadcopter after first run or calibration
	if (!initialised) {
		static uint8_t initStatus = 0;

		switch (initStatus) {
			case 0:
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
				// estimate initial barometer altitude
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

	roll_rate = gx_rps * DEG_PER_RAD;
	pitch_rate = gy_rps * DEG_PER_RAD;
	yaw_rate = gz_rps * DEG_PER_RAD;

	// * calculate flight setpoints, manipulated variables and control motors, when armed
	if (motors.getState() == MotorsQuad::State::armed) {
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
				else if (fMode == FlightMode::ReturnToLaunch) {
					// reset rtl state
					rtlState = RtlState::Init;
				}
			}

			switch (fMode) {
				case FlightMode::Stabilize:
					throttle_out = throttle;
					break;

				case FlightMode::AltitudeHold:
					if (rc_channelValue[THROTTLE] < THROTTLE_DEADZONE_BOT) {
						// shape vertical velocity setpoint from rc input for upwards velocity
						velocity_v_sp = shape_velocity(map(rc_channelValue[THROTTLE], 1000, THROTTLE_DEADZONE_BOT, -VELOCITY_V_LIMIT, 0), ACCEL_V_LIMIT, velocity_v_sp, dt_s);

						altitude_sp = altitude;
					}
					else if (rc_channelValue[THROTTLE] > THROTTLE_DEADZONE_TOP) {
						// shape vertical velocity setpoint from rc input for downwards velocity
						velocity_v_sp = shape_velocity(map(rc_channelValue[THROTTLE], THROTTLE_DEADZONE_TOP, 2000, 0, VELOCITY_V_LIMIT), ACCEL_V_LIMIT, velocity_v_sp, dt_s);

						altitude_sp = altitude;
					}
					else {
						// shape vertical velocity setpoint to hold altitude
						velocity_v_sp = constrain(shape_position(altitude_sp - altitude, TC_ALTITUDE, ACCEL_V_LIMIT, velocity_v_sp, dt_s), -VELOCITY_V_LIMIT, VELOCITY_V_LIMIT);
					}

					// calculate manipulated variable for vertical velocity
					velocity_v_mv = velocity_v_pid.get_mv(velocity_v_sp, velocity_v, dt_s);

					// use throttle hover, so vertical velocity controller can take over smoothly
					throttle_out = THROTTLE_HOVER + velocity_v_mv;
					break;

				static uint32_t dt_state = 0;
				static const uint32_t STATE_DT_MIN = 5000000; // minimum time in microseconds the condition for switching to the next state needs to be met before switching

				case FlightMode::ReturnToLaunch:
					// rtl state machine
					switch (rtlState) {
						case RtlState::Init:
							rtl_location = current_location;
							altitude_sp = altitude;
							distance_yaw = 0;

							dt_state = 0;

							headingCorrection = 0;
							headingCorrection_rad = 0;

							rtlState = RtlState::Wait;
							break;

						case RtlState::Wait:
							// bearing to rtl location
							bearing_rad = current_location.BearingTo(rtl_location);
							bearing = bearing_rad * DEG_PER_RAD;

							// distance to rtl location
							distance = current_location.DistanceKm(rtl_location) * 1000;

							// keep the current yaw angle
							distance_yaw = 0;

							// check if the rtl location is reached
							if (distance < 1.5) {
								dt_state += dt;
								if (dt_state > STATE_DT_MIN) {
									dt_state = 0;
									rtlState = RtlState::Climb;
								}
							}
							break;

						case RtlState::Climb:
							// bearing to rtl location
							bearing_rad = current_location.BearingTo(rtl_location);
							bearing = bearing_rad * DEG_PER_RAD;

							// distance to rtl location
							distance = current_location.DistanceKm(rtl_location) * 1000;

							// keep the current yaw angle
							distance_yaw = 0;

							// climb to the maximum altitude reached during flight with some added offset for safety
							altitude_sp = altitude_max + RTL_RETURN_OFFSET;

							// check if the altitude setpoint is reached
							if (abs(altitude_sp - altitude) < 1.5) {
								dt_state += dt;
								if (dt_state > STATE_DT_MIN) {
									dt_state = 0;
									rtlState = RtlState::YawToLaunch;
								}
							}
							break;

						case RtlState::YawToLaunch:
							// bearing to rtl location
							bearing_rad = current_location.BearingTo(rtl_location);
							bearing = bearing_rad * DEG_PER_RAD;

							// distance to rtl location
							distance = current_location.DistanceKm(rtl_location) * 1000;

							// turn the quadcopter towards the launch location, but only if it is not too close
							if (current_location.DistanceKm(launch_location) * 1000 > 10) {
								// turn the quadcopter towards the launch location
								distance_yaw = current_location.BearingTo(launch_location) * DEG_PER_RAD - yaw_angle;

								// check if the quadcopter is rotated towards the launch location
								if (abs(distance_yaw) < 6) {
									dt_state += dt;
									if (dt_state > STATE_DT_MIN) {
										dt_state = 0;
										rtlState = RtlState::Return;
									}
								}
							}
							else {
								// keep the current yaw angle
								distance_yaw = 0;

								dt_state = 0;
								rtlState = RtlState::Return;
							}
							break;

						case RtlState::Return:
							// bearing to launch location
							bearing_rad = current_location.BearingTo(launch_location);
							bearing = bearing_rad * DEG_PER_RAD;

							// distance to launch location
							distance = current_location.DistanceKm(launch_location) * 1000;

							// turn the quadcopter towards the launch location and calculate heading correction, but only if it is not too close
							if (distance > 10) {
								// turn the quadcopter towards the launch location
								distance_yaw = bearing - yaw_angle;

								// calculate heading correction
								headingCorrection = ema_filter(bearing - heading, headingCorrection, EMA_HEADING_CORRECTION);
								headingCorrection_rad = headingCorrection * RAD_PER_DEG;
									
								headingCorrection_rad = 0; // TODO: Remove this line and test this.
							}
							else {
								// keep the current yaw angle
								distance_yaw = 0;
							}

							// check if the launch location is reached
							if (distance < 1.5) {
								dt_state += dt;
								if (dt_state > STATE_DT_MIN)
								{
									dt_state = 0;
									rtlState = RtlState::YawToInitial;
								}
							}
							break;

						case RtlState::YawToInitial:
							// bearing to launch location
							bearing_rad = current_location.BearingTo(launch_location);
							bearing = bearing_rad * DEG_PER_RAD;

							// distance to launch location
							distance = current_location.DistanceKm(launch_location) * 1000;

							// turn the quadcopter to the initial yaw angle determined when arming
							distance_yaw = yaw_angle_init - yaw_angle;

							// check if the initial yaw angle is reached
							if (abs(distance_yaw) < 6) {
								dt_state += dt;
								if (dt_state > STATE_DT_MIN)
								{
									dt_state = 0;
									rtlState = RtlState::Descend;
								}
							}
							break;

						case RtlState::Descend:
							// bearing to launch location
							bearing_rad = current_location.BearingTo(launch_location);
							bearing = bearing_rad * DEG_PER_RAD;

							// distance to launch location
							distance = current_location.DistanceKm(launch_location) * 1000;

							// keep the initial yaw angle determined when arming
							distance_yaw = yaw_angle_init - yaw_angle;

							// descend to the initial altitude after arming with some added offset
							altitude_sp = RTL_DESCEND_OFFSET;
							break;

						default:
							break;
					}

					// adjust the yaw distance to [-180, 180) in order to make sure the quadcopter turns the shortest way
					adjustAngleRange(-180, 180, distance_yaw);

					// transform distance from ned- to horizontal frame and include heading correction to compensate inaccurate horizontal movement caused by bad compass measurements and wind
					distance_x = distance * cos(bearing_rad - yaw_angle_rad + headingCorrection_rad);
					distance_y = distance * sin(bearing_rad - yaw_angle_rad + headingCorrection_rad);

					// shape x- and y-axis velocity setpoints
					velocity_x_sp = constrain(shape_position(distance_x, TC_DISTANCE, ACCEL_H_LIMIT, velocity_x_sp, dt_s), -VELOCITY_XY_LIMIT, VELOCITY_XY_LIMIT);
					velocity_y_sp = constrain(shape_position(distance_y, TC_DISTANCE, ACCEL_H_LIMIT, velocity_y_sp, dt_s), -VELOCITY_XY_LIMIT, VELOCITY_XY_LIMIT);

					// transform velocity from ned- to horizontal frame.
					velocity_x = velocity_north * cos(yaw_angle_rad) + velocity_east * sin(yaw_angle_rad);
					velocity_y = velocity_north * (-sin(yaw_angle_rad)) + velocity_east * cos(yaw_angle_rad);

					// shape vertical velocity setpoint
					velocity_v_sp = constrain(shape_position(altitude_sp - altitude, TC_ALTITUDE, ACCEL_V_LIMIT, velocity_v_sp, dt_s), -VELOCITY_V_LIMIT, VELOCITY_V_LIMIT);

					// calculate manipulated variable for vertical velocity
					velocity_v_mv = velocity_v_pid.get_mv(velocity_v_sp, velocity_v, dt_s);

					// use throttle hover, so vertical velocity controller can take over smoothly
					throttle_out = THROTTLE_HOVER + velocity_v_mv;
					break;

				default:
					break;
			}

			// Tilt compensated thrust: Increase throttle when the quadcopter is tilted, to compensate for height loss during horizontal movement.
			// Note: In order to maintain stability, throttle is limited to the throttle limit.
			throttle_out = constrain((float) (throttle_out - 1000) / (pose_q[0] * pose_q[0] - pose_q[1] * pose_q[1] - pose_q[2] * pose_q[2] + pose_q[3] * pose_q[3]) + 1000, 1000, THROTTLE_LIMIT);

			if (fMode == FlightMode::ReturnToLaunch) {
				// calculate angle setpoints from velocity setpoints
				roll_angle_sp = velocity_y_pid.get_mv(velocity_y_sp, velocity_y, dt_s);
				pitch_angle_sp = velocity_x_pid.get_mv(velocity_x_sp, velocity_x, dt_s);
				
				// shape yaw rate setpoint
				yaw_rate_sp = constrain(shape_position(distance_yaw, TC_YAW_ANGLE, ACCEL_YAW_LIMIT, yaw_rate_sp, dt_s), -YAW_RATE_LIMIT, YAW_RATE_LIMIT);
			}
			else {
				// update the maximum altitude for rtl
				altitude_max = max(altitude, altitude_max);

				// roll and pitch angle setpoints
				roll_angle_sp = map((float) rc_channelValue[ROLL], 1000, 2000, -ROLL_PITCH_ANGLE_LIMIT, ROLL_PITCH_ANGLE_LIMIT);
				pitch_angle_sp = map((float) rc_channelValue[PITCH], 1000, 2000, -ROLL_PITCH_ANGLE_LIMIT, ROLL_PITCH_ANGLE_LIMIT);

				// shape yaw rate setpoint
				if (rc_channelValue[THROTTLE] < 1050) {
					// if throttle is too low, disable setting a yaw rate, since it might cause problems when disarming
					yaw_rate_sp = shape_velocity(0, ACCEL_YAW_LIMIT, yaw_rate_sp, dt_s);
				} else {
					yaw_rate_sp = shape_velocity(map((float) rc_channelValue[YAW], 1000, 2000, YAW_RATE_LIMIT, -YAW_RATE_LIMIT), ACCEL_YAW_LIMIT, yaw_rate_sp, dt_s);
				}
			}

			// shape roll and pitch rate setpoints
			roll_rate_sp = shape_position(roll_angle_sp - roll_angle, TC_ROLL_PITCH_ANGLE, ACCEL_ROLL_PITCH_LIMIT, roll_rate_sp, dt_s);
			pitch_rate_sp = shape_position(pitch_angle_sp - pitch_angle, TC_ROLL_PITCH_ANGLE, ACCEL_ROLL_PITCH_LIMIT, pitch_rate_sp, dt_s);

			// TODO: Remove this test code
			static float p_rate, i_rate, d_rate;

			/*
			p_rate = constrain(map((float) rc_channelValue[4], 1000, 2000, 2.0 2.5), 2.0, 2.5);
			i_rate = constrain(map((float) rc_channelValue[5], 1000, 2000, 1.0, 2.0), 1.0, 2.0);
			//d_rate = constrain(map((float) rc_channelValue[5], 1000, 2000, 0.02, 0.03), 0.02, 0.03);

			roll_rate_pid.set_K_p(p_rate);
			roll_rate_pid.set_K_i(i_rate);
			//roll_rate_pid.set_K_d(d_rate);

			pitch_rate_pid.set_K_p(p_rate);
			pitch_rate_pid.set_K_i(i_rate);
			//pitch_rate_pid.set_K_d(d_rate);
			*/

			/*
			p_rate = constrain(map((float) rc_channelValue[4], 1000, 2000, 3.5, 4.5), 3.5, 4.5);
			i_rate = constrain(map((float) rc_channelValue[5], 1000, 2000, 2.0, 4.0), 2.0, 4.0);
			//d_rate = constrain(map((float) rc_channelValue[5], 1000, 2000, 0.02, 0.025), 0.02, 0.025);

			yaw_rate_pid.set_K_p(p_rate);
			yaw_rate_pid.set_K_i(i_rate);
			//yaw_rate_pid.set_K_d(d_rate);
			*/

			// TODO: Tune PID-controller for vertical velocity.
			p_rate = constrain(map((float) rc_channelValue[4], 1000, 2000, 100, 200), 100, 200);
			i_rate = constrain(map((float) rc_channelValue[5], 1000, 2000, 10, 50), 10, 50);
			//d_rate = constrain(map((float) rc_channelValue[5], 1000, 2000, 20, 60), 20, 60);

			velocity_v_pid.set_K_p(p_rate);
			velocity_v_pid.set_K_i(i_rate);
			//velocity_v_pid.set_K_d(d_rate);

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
		baroAltitude_init = baroAltitude;

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

		//DEBUG_PRINT(ax_filtered); DEBUG_PRINT("\t"); DEBUG_PRINT(ay_filtered); DEBUG_PRINT("\t"); DEBUG_PRINTLN(az_filtered);
		//DEBUG_PRINT(gx_rps); DEBUG_PRINT("\t"); DEBUG_PRINT(gy_rps); DEBUG_PRINT("\t"); DEBUG_PRINTLN(gz_rps);
		//DEBUG_PRINT(mx); DEBUG_PRINT("\t"); DEBUG_PRINT(my); DEBUG_PRINT("\t"); DEBUG_PRINTLN(mz);

		//static float roll_angle_accel, pitch_angle_accel;
		//accelAngles(roll_angle_accel, pitch_angle_accel);
		//DEBUG_PRINT(roll_angle_accel); DEBUG_PRINT("\t"); DEBUG_PRINTLN(pitch_angle_accel);

		//DEBUG_PRINT(map((float) rc_channelValue[ROLL], 1000, 2000, -ROLL_PITCH_ANGLE_LIMIT, ROLL_PITCH_ANGLE_LIMIT)); DEBUG_PRINT("\t"); DEBUG_PRINTLN(roll_angle_sp);
		//DEBUG_PRINT(map((float) rc_channelValue[YAW], 1000, 2000, -YAW_RATE_LIMIT, YAW_RATE_LIMIT)); DEBUG_PRINT("\t"); DEBUG_PRINTLN(yaw_rate_sp);
		//DEBUG_PRINT(map((float) rc_channelValue[THROTTLE], 1000, 2000, 1000, THROTTLE_LIMIT)); DEBUG_PRINT("\t"); DEBUG_PRINTLN(throttle_out);
		//DEBUG_PRINT(map((float) (rc_channelValue[THROTTLE] - 1000) / (cos(roll_angle * RAD_PER_DEG) * cos(pitch_angle * RAD_PER_DEG)) + 1000, 1000, 2000, 1000, THROTTLE_LIMIT)); DEBUG_PRINT("\t"); DEBUG_PRINTLN(throttle_out);

		//DEBUG_PRINTLN(roll_rate_sp);

		//DEBUG_PRINT(roll_angle); DEBUG_PRINT("\t"); DEBUG_PRINT(pitch_angle); DEBUG_PRINT("\t"); DEBUG_PRINT(yaw_angle); DEBUG_PRINT("\t");
		//DEBUG_PRINTLN(yaw_rate_sp);
		//DEBUG_PRINT(roll_rate); DEBUG_PRINT("\t"); DEBUG_PRINT(pitch_rate); DEBUG_PRINT("\t"); DEBUG_PRINTLN(yaw_rate);

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
	static float yaw_angle_last;

	switch (initPose_state) {
		case state::init:
			DEBUG_PRINTLN(F("Estimating initial pose. Keep device at rest ..."));

			yaw_angle_last = yaw_angle - 1000;
			
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
			&& ((abs(yaw_angle - yaw_angle_last) / dt_s) < init_rate)) {
				// reduce beta value, since filtered angles have stabilized during initialisation
				madgwickFilter.set_beta(beta);

				initPose_state = state::init;

				DEBUG_PRINTLN(F("Initial pose estimated."));
				DEBUG_PRINTLN2(abs(roll_angle_accel - roll_angle), 6);
				DEBUG_PRINTLN2(abs(pitch_angle_accel - pitch_angle), 6);
				DEBUG_PRINTLN2(abs(yaw_angle - yaw_angle_last) / dt_s, 6);

				return true;
			}
			yaw_angle_last = yaw_angle;

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
	roll_angle_accel = atan2(ay_filtered, az_filtered) * DEG_PER_RAD;
	pitch_angle_accel = atan2(-ax_filtered, sqrt(pow(ay_filtered, 2) + pow(az_filtered, 2))) * DEG_PER_RAD;
}

// estimate initial barometer altitude
bool initAltitude(float init_velocity_v, float dt_s) {
	static state initAltitude_state = state::init;
	static float dt_altitude_s;
	static float baroAltitude_last;

	switch (initAltitude_state) {
		case state::init:
			DEBUG_PRINTLN(F("Estimating initial barometer altitude. Keep device at rest ..."));

			dt_altitude_s = 0;
			baroAltitude_last = baroAltitude - 1000;

			initAltitude_state = state::busy;
			break;

		case state::busy:
			if (dt_altitude_s > 1) {
				// initial altitude is estimated if vertical velocity converged
				// TODO: Use velocity from altitudeFilter instead
				if ((abs(baroAltitude - baroAltitude_last) / dt_altitude_s) < init_velocity_v) {
					initAltitude_state = state::init;

					DEBUG_PRINTLN(F("Initial barometer altitude estimated."));
					DEBUG_PRINTLN2(abs(baroAltitude - baroAltitude_last) / dt_altitude_s, 2);

					return true;
				}
				baroAltitude_last = baroAltitude;
				dt_altitude_s = 0;

				DEBUG_PRINTLN(baroAltitude);
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
	a_q[0] = 0; a_q[1] = ax_filtered; a_q[2] = ay_filtered; a_q[3] = az_filtered;

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
