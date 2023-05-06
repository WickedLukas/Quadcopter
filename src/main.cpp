#include "main.h"
#include "calibration.h"
#include "motorsQuad.h"
#include "shapeTrajectory.h"
#include "DataLogger.h"
#include "sendSerial.h"

#include <iBus.h>
#include <ICM20948.h>
#include <MadgwickAHRS.h>
#include <ema_filter.h>
#include <PID_controller.h>
#include <BMP388_DEV.h>

#include <Arduino.h>
#include <EEPROM.h>

// ! Motors are disabled when MOTORS_OFF is defined inside "common.h".
// ! Debug options can be enabled within "common.h".
// ! Parameters can be set inside "main.h".

#ifdef PLOT
Plotter plot;
#endif

// radio control (rc)
IBUS rc;

// inertial measurement unit (imu)
ICM20948_SPI imu(IMU_CS_PIN, IMU_SPI_PORT);

// imu interrupt
volatile bool imuInterrupt = false;
void imuReady() {
	imuInterrupt = true;
}

// Madgwick filter for pose estimation
MADGWICK_AHRS madgwickFilter(BETA_INIT);

#ifdef USE_BAR
// barometer
BMP388_DEV barometer;

// barometer interrupt
volatile bool barometerInterrupt = false;
void barometerReady() {
	barometerInterrupt = true;
}
#endif

#ifdef USE_GPS
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
#endif

// motors
MotorsQuad motors(MOTOR_PIN_1, MOTOR_PIN_2, MOTOR_PIN_3, MOTOR_PIN_4, MOTOR_PWM_RESOLUTION, MOTOR_PWM_FREQUENCY);

// rate PID controller
PID_controller roll_rate_pid(P_ROLL_RATE, I_ROLL_RATE, D_ROLL_RATE, 350, 100, EMA_ROLL_RATE_P, EMA_ROLL_RATE_D, true);
PID_controller pitch_rate_pid(P_PITCH_RATE, I_PITCH_RATE, D_PITCH_RATE, 350, 100, EMA_PITCH_RATE_P, EMA_PITCH_RATE_D, true);
PID_controller yaw_rate_pid(P_YAW_RATE, I_YAW_RATE, D_YAW_RATE, 250, 100, EMA_YAW_RATE_P, EMA_YAW_RATE_D, true);

// vertical velocity PID controller for altitude hold
PID_controller velocity_v_pid(P_VELOCITY_V, I_VELOCITY_V, D_VELOCITY_V, THROTTLE_LIMIT - THROTTLE_HOVER, THROTTLE_LIMIT - THROTTLE_HOVER, EMA_VELOCITY_V_P, EMA_VELOCITY_V_D);

// horizontal velocity PID controllers for return to launch
PID_controller velocity_x_pid(P_VELOCITY_H, I_VELOCITY_H, D_VELOCITY_H, ROLL_PITCH_ANGLE_LIMIT, ROLL_PITCH_ANGLE_LIMIT, EMA_VELOCITY_H_P, EMA_VELOCITY_H_D);
PID_controller velocity_y_pid(P_VELOCITY_H, I_VELOCITY_H, D_VELOCITY_H, ROLL_PITCH_ANGLE_LIMIT, ROLL_PITCH_ANGLE_LIMIT, EMA_VELOCITY_H_P, EMA_VELOCITY_H_D);

#ifdef USE_SDLOG
// SD card logger
DataLogger sdCardLogger("0_0_1 - connection 0 - TCP", 10'000);
#endif

// flight modes
enum class FlightMode : unsigned char { Stabilize, AltitudeHold, ReturnToLaunch } fMode;

// return to launch states
enum class RtlState : unsigned char { Init, Wait, Climb, YawToLaunch, Return, YawToInitial, Descend } rtlState;

// calibration data
calibration_data calibration_eeprom;

int32_t dt; // loop time in microseconds
float dt_s; // loop time in seconds

// flags used to initialise the quadcopter pose and altitude after power on or calibration
bool initialiseQuad = true;
bool quadInitialised = false;

// maximum quadcopter altitude relative to starting ground
float altitude_max;

// Stores which errors occurred. Each bit belongs to a certain error.
// If the error code is unequal zero, arming is disabled.
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

// raw barometer altitude
float baroAltitudeRaw;

// ema filtered raw barometer altitude
float baroAltitude;

// last quadcopter barometer altitude when disarmed
float baroAltitude_init;

// altitude relative to ground (altitude when disarmed)
float altitude;

// vertical velocity
float velocity_v;

// velocities in ned-frame
float velocity_north, velocity_east;

// velocities in horizontal frame
float velocity_x, velocity_y; 

// initial quadcopter yaw (z-axis) angle
float yaw_angle_init;

// distance to target location
float distance;

// heading
float heading;

// filtered gyro rates
float roll_rate, pitch_rate, yaw_rate;

// PID calculations are delayed until started flag becomes true, which happens when minimum throttle is reached
bool started = false;

void setup() {
	// setup built in LED
	pinMode(LED_PIN, OUTPUT);

	// turn off LED
	updateLed(LED_PIN, 0);

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
		DEBUG_PRINTLN(F("IMU Initialisation failed."));
	}

	// read accelerometer resolution in g/bit
	imu.read_accelRes(accelRes);

	// attach imu interrupt
	attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), imuReady, FALLING);

#ifdef USE_BAR
	// initialise barometer with mode, pressure oversampling, temperature oversampling, IIR-Filter and standby time
	if (!barometer.begin(NORMAL_MODE, OVERSAMPLING_SKIP, OVERSAMPLING_SKIP, IIR_FILTER_32, TIME_STANDBY_5MS)) {
		// barometer could not be initialised
		error_code |= ERROR_BAR; // set error value to disable arming
		DEBUG_PRINTLN(F("Barometer initialisation failed."));
	}

	// setup barometer interrupt pin
	pinMode(BAROMETER_INTERRUPT_PIN, INPUT);
	barometer.enableInterrupt();

	attachInterrupt(digitalPinToInterrupt(BAROMETER_INTERRUPT_PIN), barometerReady, RISING);
#endif

#ifdef PLOT
	// start plotter
	plot.Begin();

	addTimeGraphs(plot);
#endif
}

void loop() {

	// update LED according to quadcopter status (error, motors armed/disarmed and initialisation)
	updateLedStatus();

	// update radio control (rc)
	rc.update();

	// perform calibration if motors are disarmed and rc calibration request was received
	if (calibration(motors, imu, rc_channelValue, calibration_eeprom)) {
		// reinitialise quadcopter, because calibration was performed
		initialiseQuad = true;
	}

	// arm/disarm on rc command or disarm on failsafe conditions
	arm_failsafe(FS_CONFIG);

	// update time
	static uint32_t t;
	t = micros();
	static uint32_t t0 = t;

	dt = (t - t0);      // in us

	// continue if imu interrupt has fired
	if (!imuInterrupt) {
		return;
	}
	imuInterrupt = false; // reset imu interrupt

	// ! Workaround for inconsistencies in update time caused by other sensor updates.
	static const int32_t IMU_INTERRUPT_INTERVAL_US = 111; // ! ICM20948 update interval.
	static const int32_t IMU_INTERRUPT_INTERVAL_TOLERANCE = IMU_INTERRUPT_INTERVAL_US / 20;
	int32_t dt_error = dt - IMU_INTERRUPT_INTERVAL_US;
	if (abs(dt_error) < IMU_INTERRUPT_INTERVAL_TOLERANCE) {
		// normal, so there is nothing to do
	}
	else if (dt_error > IMU_INTERRUPT_INTERVAL_TOLERANCE) {
		// update too late
		int32_t rest = dt % IMU_INTERRUPT_INTERVAL_US;
		dt = dt - rest;
		t = t - rest;
	}
	else {
		// update too early, so sensor measurements were not updated yet
		return;
	}
	dt_s = dt * 1.e-6f; // in s
	
	t0 = t;

	// read accel and gyro measurements
	imu.read_accel_gyro_rps(ax, ay, az, gx_rps, gy_rps, gz_rps);

	// filter accelerometer measurements
	ax_filtered = ema_filter(ax, ax_filtered, EMA_ACCEL);
	ay_filtered = ema_filter(ay, ay_filtered, EMA_ACCEL);
	az_filtered = ema_filter(az, az_filtered, EMA_ACCEL);

#ifdef USE_MAG
	getMagData(mx, my, mz);
#endif

	// calculate pose from sensor data using Madgwick filter
	madgwickFilter.get_euler_quaternion(dt_s, ax_filtered, ay_filtered, az_filtered, gx_rps, gy_rps, gz_rps, mx, my, mz, roll_angle, pitch_angle, yaw_angle, pose_q);

	// convert to rate in deg/s
	roll_rate = gx_rps * DEG_PER_RAD;
	pitch_rate = gy_rps * DEG_PER_RAD;
	yaw_rate = gz_rps * DEG_PER_RAD;

	// apply offset to z-axis pose in order to compensate for the sensor mounting orientation relative to the quadcopter frame
	yaw_angle += YAW_ANGLE_OFFSET;
#ifdef USE_GPS
	// match yaw angle and gps heading orientation
	yaw_angle = -yaw_angle;
#endif
	adjustAngleRange(0, 360, yaw_angle);
	yaw_angle_rad = yaw_angle * RAD_PER_DEG;

#ifdef USE_BAR
	getBarData(baroAltitudeRaw);

	// filter raw barometer altitude measurement
	baroAltitude = ema_filter(baroAltitudeRaw, baroAltitude, EMA_ALT);

	// calculate vertical velocity
	static float baroAltitude_last = baroAltitude;
	velocity_v = (baroAltitude - baroAltitude_last) / dt_s;
	baroAltitude_last = baroAltitude;

	// altitude relative to starting ground
	altitude = baroAltitude - baroAltitude_init;
#endif

#ifdef USE_GPS
	getGpsData(launch_location, current_location, heading, velocity_north, velocity_east);
#endif

	// initialise quadcopter (pose, altitude) after first run or calibration
	quadInitialised = initQuad(initialiseQuad);

	// calculate flight setpoints, manipulated variables and control motors, when armed
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

		static float throttle;
		// map throttle to [-1, 1]
		throttle = map((float) rc_channelValue[THROTTLE], 1000, 2000, -1, 1);
		// apply expo to throttle for less sensitivity around hover
		throttle = expo_curve(throttle, THROTTLE_EXPO);
		// map throttle through THROTTLE_ARMED, THROTTLE_HOVER and THROTTLE_LIMIT
		throttle = map3(throttle, -1, 0, 1, THROTTLE_ARMED, THROTTLE_HOVER, THROTTLE_LIMIT);

		// in order to ensure a smooth start, PID calculations are delayed until hover throttle is reached
		if (started) {
			// last flight mode
			static FlightMode fMode_last{fMode};

#if defined(USE_GPS) && defined(USE_BAR) && defined(USE_MAG)
			if ((rc_channelValue[RTL] == 2000) && !(error_code & (ERROR_GPS | ERROR_BAR))) {
				fMode = FlightMode::ReturnToLaunch;
			}
#endif

			if (fMode != fMode_last) {
				if ((fMode_last == FlightMode::AltitudeHold) || (fMode_last == FlightMode::ReturnToLaunch)) {
					if ((fMode != FlightMode::AltitudeHold) && (fMode != FlightMode::ReturnToLaunch))
					{
						velocity_v_sp = 0;
						velocity_v_pid.reset();
					}

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
				
				fMode_last = fMode;
			}

			switch (fMode) {
				case FlightMode::Stabilize:
					// calculate roll and pitch angle setpoints as well as yaw rate setpoint from radio control input
					rc_rpAngle_yRate(roll_angle_sp, pitch_angle_sp, yaw_rate_sp);

					throttle_out = throttle;

					// update the maximum altitude for rtl
					altitude_max = max(altitude, altitude_max);
					break;

				case FlightMode::AltitudeHold:
					// calculate roll and pitch angle setpoints as well as yaw rate setpoint from radio control input
					rc_rpAngle_yRate(roll_angle_sp, pitch_angle_sp, yaw_rate_sp);

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
					throttle_out = THROTTLE_HOVER + velocity_v_mv; // use throttle hover, so vertical velocity controller can take over smoothly

					// update the maximum altitude for rtl
					altitude_max = max(altitude, altitude_max);
					break;
#ifdef USE_GPS
				case FlightMode::ReturnToLaunch:
					// calculate xyv-velocity setpoints and yaw rate setpoint for returning to launch
					rtl_xyVelocity_yRate(velocity_x_sp, velocity_y_sp, velocity_v_sp, yaw_rate_sp);

					// transform velocity from ned- to horizontal frame.
					velocity_x = velocity_north * cos(yaw_angle_rad) + velocity_east * sin(yaw_angle_rad);
					velocity_y = velocity_north * (-sin(yaw_angle_rad)) + velocity_east * cos(yaw_angle_rad);

					// calculate angle setpoints from velocity setpoints
					roll_angle_sp = velocity_y_pid.get_mv(velocity_y_sp, velocity_y, dt_s);
					pitch_angle_sp = velocity_x_pid.get_mv(velocity_x_sp, velocity_x, dt_s);

					// calculate manipulated variable for vertical velocity
					velocity_v_mv = velocity_v_pid.get_mv(velocity_v_sp, velocity_v, dt_s);
					throttle_out = THROTTLE_HOVER + velocity_v_mv; // use throttle hover, so vertical velocity controller can take over smoothly
					break;
#endif
				default:
					break;
			}

			// Tilt compensated thrust: Increase throttle when the quadcopter is tilted, to compensate for height loss during horizontal movement.
			// Note: In order to maintain stability, throttle is limited to the throttle limit.
			throttle_out = constrain((float) (throttle_out - 1000) / (pose_q[0] * pose_q[0] - pose_q[1] * pose_q[1] - pose_q[2] * pose_q[2] + pose_q[3] * pose_q[3]) + 1000, 1000, THROTTLE_LIMIT);

			// shape roll and pitch rate setpoints
			roll_rate_sp = shape_position(roll_angle_sp - roll_angle, TC_ROLL_PITCH_ANGLE, ACCEL_ROLL_PITCH_LIMIT, roll_rate_sp, dt_s);
			pitch_rate_sp = shape_position(pitch_angle_sp - pitch_angle, TC_ROLL_PITCH_ANGLE, ACCEL_ROLL_PITCH_LIMIT, pitch_rate_sp, dt_s);

			// TODO: Remove this test code
			//static float p_rate, i_rate, d_rate;

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
			//p_rate = constrain(map((float) rc_channelValue[4], 1000, 2000, 40, 100), 40, 100);
			//i_rate = constrain(map((float) rc_channelValue[5], 1000, 2000, 10, 20), 10, 20);
			//d_rate = constrain(map((float) rc_channelValue[5], 1000, 2000, 20, 60), 20, 60);

			//velocity_v_pid.set_K_p(p_rate);
			//velocity_v_pid.set_K_i(i_rate);
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
				DEBUG_PRINTLN(F("Started."));
			}
		}
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

#ifdef USE_SDLOG
	static bool sdLogEnabled{false};
	// start/stop SD card logging
	if ((sdLogEnabled == false) && ((rc_channelValue[ARM] == 2000) && (quadInitialised == true))) {
		sdLogEnabled = true;
		if (!sdCardLogger.start()) {
			error_code |= ERROR_SDLOG;
		}
		else {
			error_code &= !ERROR_SDLOG;

			DEBUG_PRINTLN("SD card logging started.");
		}
	}
	else if ((sdLogEnabled == true) && ((rc_channelValue[ARM] == 1000) || (quadInitialised == false))) {
		sdLogEnabled = false;
		if (!sdCardLogger.stop()) {
			error_code |= ERROR_SDLOG;
		}
		DEBUG_PRINTLN("SD card logging stopped.");
	}

	SD_LOG2(fMode, static_cast<unsigned char>(fMode)); SD_LOG2(rtlState, static_cast<unsigned char>(rtlState));
	SD_LOG(dt);
	SD_LOG(ax); SD_LOG(ay); SD_LOG(az);
	SD_LOG2D(ax_filtered, 1); SD_LOG2D(ay_filtered, 1); SD_LOG2D(az_filtered, 1);
	SD_LOG2D(roll_angle, 2); SD_LOG2D(pitch_angle, 2); SD_LOG2D(yaw_angle, 2); SD_LOG2D(roll_angle_sp, 2); SD_LOG2D(pitch_angle_sp, 2);
	SD_LOG2D(roll_rate, 2); SD_LOG2D(pitch_rate, 2); SD_LOG2D(yaw_rate, 2); SD_LOG2D(roll_rate_sp, 2); SD_LOG2D(pitch_rate_sp, 2); SD_LOG2D(yaw_rate_sp, 2);
	SD_LOG2D(yaw_rate, 2); SD_LOG2D(yaw_rate_sp, 2);
	SD_LOG2D(baroAltitudeRaw, 2); SD_LOG2D(baroAltitude, 2);
	SD_LOG2D(altitude, 2); SD_LOG2D(altitude_sp, 2);
	SD_LOG2D(velocity_v, 2); SD_LOG2D(velocity_x, 2); SD_LOG2D(velocity_y, 2); SD_LOG2D(velocity_v_sp, 2); SD_LOG2D(velocity_x_sp, 2); SD_LOG2D(velocity_y_sp, 2);
	SD_LOG2D(throttle_out, 0);
	SD_LOG3(roll_rate_pTerm, roll_rate_pid.get_pTerm(), 2); SD_LOG3(roll_rate_iTerm, roll_rate_pid.get_iTerm(),  2); SD_LOG3(roll_rate_dTerm, roll_rate_pid.get_dTerm(), 2);
	SD_LOG3(pitch_rate_pTerm, pitch_rate_pid.get_pTerm(), 2); SD_LOG3(pitch_rate_iTerm, pitch_rate_pid.get_iTerm(), 2); SD_LOG3(pitch_rate_dTerm, pitch_rate_pid.get_dTerm(), 2);
	SD_LOG3(yaw_rate_pTerm, yaw_rate_pid.get_pTerm(), 2); SD_LOG3(yaw_rate_iTerm, yaw_rate_pid.get_iTerm(), 2); SD_LOG3(yaw_rate_dTerm, yaw_rate_pid.get_dTerm(), 2);
	SD_LOG3(velocity_v_pTerm, velocity_v_pid.get_pTerm(), 2); SD_LOG3(velocity_v_iTerm, velocity_v_pid.get_iTerm(), 2); SD_LOG3(velocity_v_dTerm, velocity_v_pid.get_dTerm(), 2);
	SD_LOG3(velocity_x_pTerm, velocity_x_pid.get_pTerm(), 2); SD_LOG3(velocity_x_iTerm, velocity_x_pid.get_iTerm(), 2); SD_LOG3(velocity_x_dTerm, velocity_x_pid.get_dTerm(), 2);
	SD_LOG3(velocity_y_pTerm, velocity_y_pid.get_pTerm(), 2); SD_LOG3(velocity_y_iTerm, velocity_y_pid.get_iTerm(), 2); SD_LOG3(velocity_y_dTerm, velocity_y_pid.get_dTerm(), 2);


	// write log line to file
	if (!sdCardLogger.writeLogLine()) {
		sdCardLogger.stop();
		error_code |= ERROR_SDLOG;
	}
#endif

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

		//DEBUG_PRINT(baroAltitudeRaw); DEBUG_PRINT("\t"); DEBUG_PRINTLN(baroAltitude);

		//DEBUG_PRINTLN(dt);
		//DEBUG_PRINTLN();

		// print channel values
		/*for (int i=0; i<10 ; i++) {
			DEBUG_PRINT(rc_channelValue[i]); DEBUG_PRINT("\t");
		}
		DEBUG_PRINTLN();*/
	}
#elif defined(PLOT)
	// TODO: Check if this update rate is still too fast
	if (micros() - t0_serial > 25000) {
		t0_serial = micros();
		// Plot data with "Processing"
		plot.Plot();
	}
#elif defined(SEND_SERIAL)
	if (micros() - t0_serial > 16666) {
		t0_serial = micros();
		// Visualize data with "Processing"
		sendSerial(dt, roll_angle, pitch_angle, yaw_angle);
	}
#endif
}

// estimate initial pose
bool initPose(float beta_init, float beta, float init_angleDifference, float init_rate) {
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

// update LED according to quadcopter status (error, motors armed/disarmed and initialisation)
void updateLedStatus()
{
	if (motors.getState() == MotorsQuad::State::armed) {
		// blink LED normally to indicate armed status
		updateLed(LED_PIN, 1, 1000);
	}
	else if ((error_code & !ERROR_GPS & !ERROR_SDLOG) != 0) {
		// blink LED very fast to indicate an error occurrence except gps and data logging
		updateLed(LED_PIN, 1, 200);
	}
	else if (!quadInitialised || (error_code == ERROR_GPS)) {
		// blink LED slowly to indicate quadcopter initialisation or gps search
		updateLed(LED_PIN, 1, 600);
	}
	else if (error_code == ERROR_SDLOG) {
		// blink LED fast to indicate a data logging error
		updateLed(LED_PIN, 1, 400);
	}
	else if (motors.getState() == MotorsQuad::State::disarmed) {
		// turn off LED to indicate disarmed status
		updateLed(LED_PIN, 0);
	}
	else {
		// turn on LED to indicate arming/disarming status
		updateLed(LED_PIN, 2);
	}
}

#ifdef USE_MAG
// get magnetometer data
void getMagData(int16_t &mx, int16_t &my, int16_t &mz) {
	static uint32_t dt_mag = 0;
	if (!(error_code & ERROR_MAG) && imu.read_mag(mx, my, mz)) {
		dt_mag = 0;
	}
	else if (!initialiseQuad) {
		// check if magnetometer is still working
		dt_mag += dt;
		if ((dt_mag > SENSOR_DT_LIMIT) && !(error_code & ERROR_MAG)) {
			// Too much time has passed since the last magnetometer reading. Set error value, which will disable arming.
			error_code |= ERROR_MAG;

			mx = my = mz = 0; // error handling

			DEBUG_PRINTLN(F("Magnetometer error!"));
		}
	}
}
#endif

#ifdef USE_BAR
// get barometer data
void getBarData(float &baroAltRaw) {
	// get raw altitude from barometer
	static uint32_t dt_bar = 0;
	if (barometerInterrupt) {
		barometerInterrupt = false;
		dt_bar = 0;

		barometer.getAltitude(baroAltRaw);
	}
	else if (!initialiseQuad) {
		// check if barometer is still working
		dt_bar += dt;
		if ((dt_bar > SENSOR_DT_LIMIT) && !(error_code & ERROR_BAR)) {
			// Too much time has passed since the last barometer reading. Set error value, which will disable arming.
			error_code |= ERROR_BAR;

			DEBUG_PRINTLN(F("Barometer error!"));
		}
	}
}
#endif

#ifdef USE_GPS
// get gps data
void getGpsData(NeoGPS::Location_t &launch_location, NeoGPS::Location_t &current_location, float &heading, float &velocity_north, float &velocity_east) {
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
	else if (!initialiseQuad) {
		// check if gps is still working
		dt_gps += dt;
		if ((dt_gps > SENSOR_DT_LIMIT) && !(error_code & ERROR_GPS)) {
			// Too much time has passed since the last gps reading. Set error value, which will disable arming.
			error_code |= ERROR_GPS;

			DEBUG_PRINTLN(F("GPS error!"));
		}
	}
}
#endif

// initialise quadcopter (pose, altitude) after first run or calibration
bool initQuad(bool &initialiseQuad) {
	static uint8_t initStatus = 0;

	if (initialiseQuad) {
		initialiseQuad = false;
		initStatus = 0;
	}

	switch (initStatus) {
		case 0:
			disarmAndResetQuad();
			++initStatus;
			break;

		case 1:
			// estimate initial pose
			if (initPose(BETA_INIT, BETA, INIT_ANGLE_DIFFERENCE, INIT_RATE)) {
				++initStatus;
			}
			break;

		default:
			// quadcopter initialisation completed
			return true;
			break;
	}
	return false;
}

// arm/disarm on rc command or disarm on failsafe conditions
void arm_failsafe(uint8_t fs_config) {
	static uint32_t t_arm_failsafe;
	t_arm_failsafe = micros();

	// automatically disarm when no start happens within 15 seconds
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

	// arm and disarm on rc command
	static uint32_t t_arm = 0, t_disarm = 0;
	if (rc_channelValue[ARM] == 2000 && quadInitialised) { // arm switch needs to be set and quadcopter needs to be initialised to enable arming
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

	// disarm on failsafe conditions
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

	roll_rate_mv = 0;
	pitch_rate_mv = 0;
	yaw_rate_mv = 0;

	// disarm motors
	motors.disarm();

	// set started state to false - minimum throttle is required again to start the motors and PID calculations
	started = false;
}

// calculate roll and pitch angle setpoints as well as yaw rate setpoint from radio control input
void rc_rpAngle_yRate(float &roll_angle_sp, float &pitch_angle_sp, float &yaw_rate_sp)
{
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

#ifdef USE_GPS
// calculate xyv-velocity setpoints and yaw rate setpoint for returning to launch
void rtl_xyVelocity_yRate(float &velocity_x_sp, float &velocity_y_sp, float &velocity_v_sp, float &yaw_rate_sp)
{
	static uint32_t dt_state = 0;
	// minimum time in microseconds the condition for switching to the next state needs to be met before switching
	static const uint32_t STATE_DT_MIN = 5000000; 

	// bearing to target location (clockwise from north)
	static float bearing, bearing_rad;

	// distance to target yaw angle
	static float distance_yaw;

	// heading correction to compensate inaccurate horizontal movement caused by bad compass measurements and wind 
	static float headingCorrection, headingCorrection_rad;

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
			if (distance < 2) {
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
			if (abs(altitude_sp - altitude) < 2) {
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
			if (distance < 2) {
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

	// because the yaw angle is inverted with gps, while the yaw rates are not, this parameter needs to be inverted
	distance_yaw = -distance_yaw;

	// adjust the yaw distance to [-180, 180) in order to make sure the quadcopter turns the shortest way
	adjustAngleRange(-180, 180, distance_yaw);

	// transform distance from ned- to horizontal frame and include heading correction to compensate inaccurate horizontal movement caused by bad compass measurements and wind
	static float distance_x, distance_y;
	distance_x = distance * cos(bearing_rad - yaw_angle_rad + headingCorrection_rad);
	distance_y = distance * sin(bearing_rad - yaw_angle_rad + headingCorrection_rad);

	// shape x- and y-axis velocity setpoints
	velocity_x_sp = constrain(shape_position(distance_x, TC_DISTANCE, ACCEL_H_LIMIT, velocity_x_sp, dt_s), -VELOCITY_XY_LIMIT, VELOCITY_XY_LIMIT);
	velocity_y_sp = constrain(shape_position(distance_y, TC_DISTANCE, ACCEL_H_LIMIT, velocity_y_sp, dt_s), -VELOCITY_XY_LIMIT, VELOCITY_XY_LIMIT);

	// shape vertical velocity setpoint
	velocity_v_sp = constrain(shape_position(altitude_sp - altitude, TC_ALTITUDE, ACCEL_V_LIMIT, velocity_v_sp, dt_s), -VELOCITY_V_LIMIT, VELOCITY_V_LIMIT);

	// shape yaw rate setpoint
	yaw_rate_sp = constrain(shape_position(distance_yaw, TC_YAW_ANGLE, ACCEL_YAW_LIMIT, yaw_rate_sp, dt_s), -YAW_RATE_LIMIT, YAW_RATE_LIMIT);
}
#endif

#ifdef PLOT
// add time graphs to plot through Processing
void addTimeGraphs(Plotter &p)
{
	// Add time graphs. Notice the effect of points displayed on the time scale.
	//p.AddTimeGraph("throttle_out", 1000, "throttle_out", throttle_out);
	//p.AddTimeGraph("Angles", 1000, "r", roll_angle, "p", pitch_angle, "y", yaw_angle);
	//p.AddTimeGraph("Rates", 1000, "roll_rate", roll_rate, "pitch_rate", pitch_rate, "yaw_rate", yaw_rate);
	//p.AddTimeGraph("mv", 1000, "roll_rate_sp", roll_rate_sp, "pitch_rate_sp", pitch_rate_sp, "yaw_rate_sp", yaw_rate_sp);
	//p.AddTimeGraph("mv", 1000, "roll_rate_mv", roll_rate_mv, "pitch_rate_mv", pitch_rate_mv, "yaw_rate_mv", yaw_rate_mv);

	//p.AddTimeGraph("alt", 1000, "A", baroAltitude, "bA", baroAltitudeRaw);
	//p.AddTimeGraph("v_vel", 1000, "V", velocity_v/*, "V_sp", velocity_v_sp*/);

	//p.AddTimeGraph("bearing", 1000, "bearing", bearing);
	//p.AddTimeGraph("distance_xy", 1000, "d_x", distance_x, "d_y", distance_y);
	//p.AddTimeGraph("velocity", 1000, "v_n", velocity_north, "v_e", velocity_east, "v_x", velocity_x, "v_y", velocity_y);

	//p.AddTimeGraph("yawTune", 1000, "yaw_rate_sp", yaw_rate_sp, "yaw_rate", yaw_rate, "yaw_angle", yaw_angle);
}
#endif
