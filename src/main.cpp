#include "main.h"
#include "common.h"
#include "calibration.h"
#include "MotorsQuad.h"
#include "shapeTrajectory.h"
#include "KalmanFilter1D.h"
#include "sendSerial.h"

#include "iBus.h"
#include "ICM20948.h"
#include "BMP388_DEV.h"
#include "MadgwickAHRS.h"
#include "ema_filter.h"
#include "PID_controller.h"
#include "Plotter.h"

#include <Arduino.h>
#include <EEPROM.h>

// Stores the errors which occurred and disables arming.
// A restart is required to reset the error and enable arming.
uint8_t error_code;

// calibration data
calibration_data calibration_eeprom;

// radio control (rc) object
IBUS rc;

// ICM-20948 imu object
ICM20948_SPI imu(IMU_CS_PIN, IMU_SPI_PORT);

// BMP388 object
BMP388_DEV bmp388;

// initial quadcopter z-axis angle - this should be initialised with an implausible value
float yaw_angle_init = -1000;

// initial quadcopter altitude - this should be initialised with an implausible value
float altitude_init = -1000;

// pointer on an array of 10 received rc channel values [1000; 2000]
uint16_t *rc_channelValue;

// flight modes
enum class FlightMode { Stabilize, TiltCompensation, AltitudeHold } fmode;

// motor objects
MotorsQuad motors(MOTOR_PIN_1, MOTOR_PIN_2, MOTOR_PIN_3, MOTOR_PIN_4, MOTOR_PWM_RESOLUTION, MOTOR_PWM_FREQENCY);

// Madgwick filter object
MADGWICK_AHRS madgwickFilter(BETA_INIT);

// Kalman-Filter 
KalmanFilter1D altitudeFilter(TC_ALTITUDE_FILTER);

// rate PID controller
PID_controller roll_rate_pid(P_ROLL_RATE, I_ROLL_RATE, D_ROLL_RATE, 0, 0, 250, 50, EMA_ROLL_RATE_P, EMA_ROLL_RATE_D);
PID_controller pitch_rate_pid(P_PITCH_RATE, I_PITCH_RATE, D_PITCH_RATE, 0, 0, 250, 50, EMA_PITCH_RATE_P, EMA_PITCH_RATE_D);
PID_controller yaw_rate_pid(P_YAW_RATE, I_YAW_RATE, D_YAW_RATE, 0, 0, 250, 100, EMA_YAW_RATE_P, EMA_YAW_RATE_D);

// vertical velocity PID controller for altitude hold
PID_controller velocity_v_pid(P_VELOCITY_V, I_VELOCITY_V, D_VELOCITY_V, 0, 0, 250, 250, EMA_velocity_v_P, EMA_velocity_v_D);

// variables to measure imu update time
uint32_t t0, t;
// measured imu update time in microseconds
int32_t dt;
// measured imu update time in seconds
float dt_s;

// throttle output
float throttle_out;

// flight setpoints
float roll_angle_sp, pitch_angle_sp;
float roll_rate_sp, pitch_rate_sp, yaw_rate_sp;
float altitude_sp;
float velocity_v_sp;

// manipulated variables
float roll_rate_mv, pitch_rate_mv, yaw_rate_mv;
float velocity_v_mv;

// accelerometer resolution
float accelRes;

// imu measurements
int16_t ax, ay, az;
float gx_rps, gy_rps, gz_rps;
int16_t mx, my, mz;

// barometer altitude measurement
float baroAltitude;

// quadcopter pose
float roll_angle, pitch_angle, yaw_angle;	// euler angles
float pose_q[4];	// quaternion

// quadcopter altitude
float altitude;

// quadcopter vertical velocity
float velocity_v;

// z-axis pose offset to compensate for the sensor mounting orientation relative to the quadcopter frame
float yaw_angle_offset = 90;

// filtered gyro rates
float roll_rate, pitch_rate, yaw_rate;

// starts PID calculation when minimum throttle setpoint was reached
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
	
	// set default resolution for analog write, in order to go back to it after running motors with different resolution
	analogWriteResolution(8);
	
	#if defined(DEBUG) || defined(SEND_SERIAL)
		// initialise serial for monitoring
		Serial.begin(115200);
		while (!Serial);
	#endif
	
	// initialise serial2 for iBus communication
	Serial2.begin(115200);
	while (!Serial2);
	
	// initialise rc and return a pointer on the received rc channel values
	rc_channelValue = rc.begin(Serial2);
	
	// initialise SPI for imu communication
	IMU_SPI_PORT.begin();
	
	// setup interrupt pin for imu
	pinMode(IMU_INTERRUPT_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), imuReady, FALLING);
	
	// get calibration data from EEPROM
	EEPROM.get(ADDRESS_EEPROM, calibration_eeprom);
	
	// initialise imu
	if (!imu.init(calibration_eeprom.offset_gx_1000dps, calibration_eeprom.offset_gy_1000dps, calibration_eeprom.offset_gz_1000dps, calibration_eeprom.offset_ax_32g, calibration_eeprom.offset_ay_32g, calibration_eeprom.offset_az_32g, calibration_eeprom.offset_mx, calibration_eeprom.offset_my, calibration_eeprom.offset_mz, calibration_eeprom.scale_mx, calibration_eeprom.scale_my, calibration_eeprom.scale_mz)) {
		// IMU could not be initialised. Set error value, which will disable arming.
		error_code = error_code | ERROR_IMU;
		DEBUG_PRINTLN("IMU error: Initialisation failed!");
	}
	
	// read accelerometer resolution in g/bit
	imu.read_accelRes(accelRes);
	
	// initialise BMP388 mode, pressure oversampling, temperature oversampling, IIR-Filter and standby time
	if (!bmp388.begin(NORMAL_MODE, OVERSAMPLING_SKIP, OVERSAMPLING_SKIP, IIR_FILTER_32, TIME_STANDBY_5MS)) {
		// barometer could not be initialised. Set error value, which will disable arming.
		error_code = error_code | ERROR_BAR;
		DEBUG_PRINTLN("BAR error: Initialisation failed!");
	}
	
	// setup interrupt pin for BMP388
	pinMode(BAROMETER_INTERRUPT_PIN, INPUT);
	bmp388.enableInterrupt();
	attachInterrupt(digitalPinToInterrupt(BAROMETER_INTERRUPT_PIN), barometerReady, RISING);
	
	#ifdef PLOT
		// ! Start plotter
		p.Begin();
		
		// Add time graphs. Notice the effect of points displayed on the time scale
		//p.AddTimeGraph("Angles", 1000, "r", roll_angle, "p", pitch_angle, "y", yaw_angle);
		//p.AddTimeGraph("Rates", 1000, "roll_rate", roll_rate, "pitch_rate", pitch_rate, "yaw_rate", yaw_rate);
		//p.AddTimeGraph("mv", 1000, "roll_rate_sp", roll_rate_sp, "pitch_rate_sp", pitch_rate_sp, "yaw_rate_sp", yaw_rate_sp);
		//p.AddTimeGraph("mv", 1000, "roll_rate_mv", roll_rate_mv, "pitch_rate_mv", pitch_rate_mv, "yaw_rate_mv", yaw_rate_mv);
		//p.AddTimeGraph("Altitude", 1000, "Quadcopter", altitude, "Barometer", baroAltitude);
		//p.AddTimeGraph("Relative acceleration in ned-frame", 1000, "a_ned_rel_q1", a_ned_rel_q1, "a_ned_rel_q2", a_ned_rel_q2, "a_ned_rel_q3", a_ned_rel_q3);
		//p.AddTimeGraph("Quadcopter vertical velocity", 1000, "velocity_v", velocity_v, "velocity_v_sp", velocity_v_sp);
		//p.AddTimeGraph("alt", 1000, "a", altitude, "bA", baroAltitude, "aS", altitude_sp);
		//p.AddTimeGraph("vel", 1000, "v", velocity_v, "v_sp", velocity_v_sp);
	#endif
}

void loop() {
	if (error_code != 0) {
		// blink LED very fast to indicate an error occurrence
		updateLED(LED_PIN, 1, 250);
	}
	else if (motors.getState() == MotorsQuad::State::armed) {
		// blink LED normally to indicate armed status
		updateLED(LED_PIN, 1, 1000);
	}
	else if (motors.getState() == MotorsQuad::State::disarmed) {
		// turn off LED to indicate disarmed status
		updateLED(LED_PIN, 0);
	}
	else {
		// turn on LED to indicate arming/disarming status
		updateLED(LED_PIN, 2);
	}
	
	// update rc
	rc.update();
	
	static bool initSensors = true;
	if (initSensors) {
		// estimate initial pose
		estimatePose(BETA_INIT, BETA, INIT_ANGLE_DIFFERENCE, INIT_RATE);
		// estimate initial altitude
		estimateAltitude(INIT_VELOCITY_V);
		
		initSensors = false;
	}
	
	// if disarmed, check for calibration request from rc and execute it
	if (calibration(motors, imu, rc_channelValue, calibration_eeprom)) {
		// calibration was performed and sensors need to be initialised again
		initSensors = true;
		return;
	}
	
	// arm/disarm on rc command or disarm on failsafe conditions
	arm_failsafe(FS_CONFIG);
	
	// update time
	t = micros();
	dt = (t - t0);  // in us
	dt_s = (float) (dt) * 1.e-6;	// in s
	
	// continue if imu interrupt has fired
	if (!imuInterrupt) {
		return;
	}
	
	// reset imu interrupt
	imuInterrupt = false;
	
	t0 = t;
	
	// read accel and gyro measurements
	imu.read_accel_gyro_rps(ax, ay, az, gx_rps, gy_rps, gz_rps);
	
	// read magnetometer measurements and check when the last ones were read
	static uint32_t dt_mag;
	if (imu.read_mag(mx, my, mz)) {
		dt_mag = 0;
	}
	else {
		dt_mag += dt;
		if ((dt_mag > FS_IMU_DT_LIMIT) && ((error_code & ERROR_MAG) != ERROR_MAG)) {
			// Limit for magnetometer update time exceeded. Set error value, which will disable arming.
			error_code = error_code | ERROR_MAG;
			DEBUG_PRINTLN("Magnetometer error!");
		}
	}
	
	if (barometerInterrupt) {  
		barometerInterrupt = false;
		bmp388.getAltitude(baroAltitude);
	}
	
	// perform sensor fusion with Madgwick filter to calculate pose
	// TODO: Check if there is a benefit from magnetometer data
	madgwickFilter.get_euler_quaternion(dt_s, ax, ay, az, gx_rps, gy_rps, gz_rps, 0, 0, 0, roll_angle, pitch_angle, yaw_angle, pose_q);
	
	// apply offset to z-axis pose in order to compensate for the sensor mounting orientation relative to the quadcopter frame
	yaw_angle += yaw_angle_offset;
	if (yaw_angle > 180) {
		yaw_angle -= 360;
	}
	else if (yaw_angle < -180) {
		yaw_angle += 360;
	}
	
	// calculate ned-acceleration relative to gravity in m/s²
	static float a_n_rel, a_e_rel, a_d_rel;
	calc_accel_ned_rel(a_n_rel, a_e_rel, a_d_rel);
	
	// get Kalman filtered altitude in m
	altitudeFilter.update(a_d_rel, baroAltitude, dt_s);
	altitude = altitudeFilter.get_position();
	velocity_v = altitudeFilter.get_velocity();
	
	roll_rate = gx_rps * RAD2DEG;
	pitch_rate = gy_rps * RAD2DEG;
	yaw_rate = gz_rps * RAD2DEG;
	
	// when armed, calculate flight setpoints, manipulated variables and control motors
	if (motors.getState() == MotorsQuad::State::armed) {
		
		// update flight mode
		if (rc_channelValue[FMODE] == 1000) {
			fmode = FlightMode::Stabilize;
		}
		else if (rc_channelValue[FMODE] == 1500) {
			fmode = FlightMode::TiltCompensation;
		}
		else {	// rc_channelValue[FMODE] == 2000
			fmode = FlightMode::AltitudeHold;
		}
		
		// remember previous flight mode
		static FlightMode fmode_last;
		
		// map throttle to [-1, 1]
		static float throttle;
		throttle = map(rc_channelValue[THROTTLE], 1000, 2000, -1, 1);
		// apply expo to throttle for less sensitivity around hover
		throttle = expo_curve(throttle, THROTTLE_EXPO);
		
		// In order to ensure a smooth start, PID calculations are delayed until hover throttle is reached.
		if (started) {
			if (fmode != fmode_last) {
				if (fmode_last == FlightMode::AltitudeHold) {
					// reset altitude hold
					velocity_v_sp = 0;
					velocity_v_pid.reset();
					velocity_v_mv = 0;
				}
				
				if (fmode == FlightMode::AltitudeHold) {
					altitude_sp = altitude;
				}
			}
			else {
				switch (fmode) {
					case FlightMode::Stabilize:
						throttle_out = map3(throttle, -1, 0, 1, THROTTLE_ARMED, THROTTLE_HOVER, THROTTLE_LIMIT);
						
						break;
					case FlightMode::TiltCompensation:
						throttle = map3(throttle, -1, 0, 1, THROTTLE_ARMED, THROTTLE_HOVER, THROTTLE_LIMIT);
						
						// Tilt compensated thrust: Increase thrust when quadcopter is tilted, to compensate for height loss during horizontal movement.
						// Note: In order to maintain stability, tilt compensated thrust is limited to the throttle limit.
						throttle_out = constrain((float) (throttle - 1000) / (pose_q[0]*pose_q[0] - pose_q[1]*pose_q[1] - pose_q[2]*pose_q[2] + pose_q[3]*pose_q[3]) + 1000, 1000, THROTTLE_LIMIT);
						
						break;
					case FlightMode::AltitudeHold:
						// if throttle stick is not centered
						if ((rc_channelValue[THROTTLE] < THROTTLE_DEADZONE_BOT) || (rc_channelValue[THROTTLE] > THROTTLE_DEADZONE_TOP)) {
							// shape rc input to control vertical velocity
							velocity_v_sp = shape_velocity(map(throttle, -1, 1, -VELOCITY_V_LIMIT, VELOCITY_V_LIMIT), ACCEL_V_MAX, velocity_v_sp, dt_s);
							
							altitude_sp = altitude;
						}
						else {
							// shape vertical velocity to hold altitude
							velocity_v_sp = constrain(shape_position(altitude_sp - altitude, TC_ALTITUDE, ACCEL_V_MAX, velocity_v_sp, dt_s), -VELOCITY_V_LIMIT, VELOCITY_V_LIMIT);
						}
						
						// fix output throttle to hover value, so vertical velocity controller can take over smoothly
						throttle_out = THROTTLE_HOVER;
						
						// calculate manipulated variable for vertical velocity
						velocity_v_mv = velocity_v_pid.get_mv(velocity_v_sp, velocity_v, dt_s);
						
						break;
					default:
						break;
				}
			}
			
			// angle setpoints
			roll_angle_sp = map((float) rc_channelValue[ROLL], 1000, 2000, -ROLL_ANGLE_LIMIT, ROLL_ANGLE_LIMIT);
			pitch_angle_sp = map((float) rc_channelValue[PITCH], 1000, 2000, -PITCH_ANGLE_LIMIT, PITCH_ANGLE_LIMIT);
			
			// rate setpoints
			roll_rate_sp = shape_position(roll_angle_sp - roll_angle, TC_ANGLE, ACCEL_MAX_ROLL_PITCH, roll_rate_sp, dt_s);
			pitch_rate_sp = shape_position(pitch_angle_sp - pitch_angle, TC_ANGLE, ACCEL_MAX_ROLL_PITCH, pitch_rate_sp, dt_s);
			
			if (rc_channelValue[THROTTLE] < 1100) {
				// if throttle is too low, disable setting a yaw rate, since it might cause problems when disarming
				yaw_rate_sp = shape_velocity(0, ACCEL_MAX_YAW, yaw_rate_sp, dt_s);
			} else {
				yaw_rate_sp = shape_velocity(map((float) rc_channelValue[YAW], 1000, 2000, YAW_RATE_LIMIT, -YAW_RATE_LIMIT), ACCEL_MAX_YAW, yaw_rate_sp, dt_s);
			}
			
			
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
			
			p_rate = constrain(map((float) rc_channelValue[4], 1000, 2000, 250, 350), 250, 350);
			i_rate = constrain(map((float) rc_channelValue[5], 1000, 2000, 5, 10), 5, 10);
			//d_rate = constrain(map((float) rc_channelValue[5], 1000, 2000, -0.001, 0.01), 0, 0.01);
			
			velocity_v_pid.set_K_p(p_rate);
			velocity_v_pid.set_K_i(i_rate);
			//yaw_rate_pid.set_K_d(0);*/
			
			// calculate manipulated variables for attitude hold
			roll_rate_mv = roll_rate_pid.get_mv(roll_rate_sp, roll_rate, dt_s);
			pitch_rate_mv = pitch_rate_pid.get_mv(pitch_rate_sp, pitch_rate, dt_s);
			yaw_rate_mv = yaw_rate_pid.get_mv(yaw_rate_sp, yaw_rate, dt_s);
		}
		else {
			throttle_out = map3(throttle, -1, 0, 1, THROTTLE_ARMED, THROTTLE_HOVER, THROTTLE_LIMIT);
			
			altitude_sp = altitude;
			
			if (throttle_out > THROTTLE_HOVER) {
				started = true;
				DEBUG_PRINTLN("Started!");
			}
		}
		
		fmode_last = fmode;
	}

	// motor mixing
	motors.output(
		constrain(throttle_out + velocity_v_mv + roll_rate_mv - pitch_rate_mv + yaw_rate_mv, 1000, 2000),
		constrain(throttle_out + velocity_v_mv - roll_rate_mv - pitch_rate_mv - yaw_rate_mv, 1000, 2000),
		constrain(throttle_out + velocity_v_mv - roll_rate_mv + pitch_rate_mv + yaw_rate_mv, 1000, 2000),
		constrain(throttle_out + velocity_v_mv + roll_rate_mv + pitch_rate_mv - yaw_rate_mv, 1000, 2000));
		
	// run serial print at a rate independent of the main loop (micros() - t0_serial = 16666 for 60 Hz update rate)
	static uint32_t t0_serial = micros();
	#ifdef DEBUG
		if (micros() - t0_serial > 16666) {
			t0_serial = micros();
			
			//DEBUG_PRINT(ax); DEBUG_PRINT("\t"); DEBUG_PRINT(ay); DEBUG_PRINT("\t"); DEBUG_PRINTLN(az);
			//DEBUG_PRINT(gx_rps); DEBUG_PRINT("\t"); DEBUG_PRINT(gy_rps); DEBUG_PRINT("\t"); DEBUG_PRINTLN(gz_rps);
			//DEBUG_PRINT(mx); DEBUG_PRINT("\t"); DEBUG_PRINT(my); DEBUG_PRINT("\t"); DEBUG_PRINTLN(mz);
			
			//static float roll_angle_accel, pitch_angle_accel;
			//accelAngles(roll_angle_accel, pitch_angle_accel);
			//DEBUG_PRINT(roll_angle_accel); DEBUG_PRINT("\t"); DEBUG_PRINTLN(pitch_angle_accel);
			
			//DEBUG_PRINT(map((float) rc_channelValue[ROLL], 1000, 2000, -ROLL_ANGLE_LIMIT, ROLL_ANGLE_LIMIT)); DEBUG_PRINT("\t"); DEBUG_PRINTLN(roll_angle_sp);
			//DEBUG_PRINT(map((float) rc_channelValue[YAW], 1000, 2000, -YAW_RATE_LIMIT, YAW_RATE_LIMIT)); DEBUG_PRINT("\t"); DEBUG_PRINTLN(yaw_rate_sp);
			//DEBUG_PRINT(map((float) rc_channelValue[THROTTLE], 1000, 2000, 1000, THROTTLE_LIMIT)); DEBUG_PRINT("\t"); DEBUG_PRINTLN(throttle_out);
			//DEBUG_PRINT(map((float) (rc_channelValue[THROTTLE] - 1000) / (cos(roll_angle * DEG2RAD) * cos(pitch_angle * DEG2RAD)) + 1000, 1000, 2000, 1000, THROTTLE_LIMIT)); DEBUG_PRINT("\t"); DEBUG_PRINTLN(throttle_out);
			
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
void estimatePose(float beta_init, float beta, float init_angleDifference, float init_rate) {
	// angles calculated from accelerometer
	float roll_angle_accel, pitch_angle_accel;

	// set higher beta value to speed up pose estimation
	madgwickFilter.set_beta(beta_init);
	
	DEBUG_PRINTLN(F("Estimating initial pose. Keep device at rest ..."));
	while (1) {
		while (!imuInterrupt) {
			// wait for next imu interrupt
		}
		// reset imu interrupt flag
		imuInterrupt = false;
		
		// blink LED fast to indicate pose estimation
		updateLED(LED_PIN, 1, 500);
		
		// update time
		t = micros();
		dt = (t - t0);	// in us
		dt_s = (float) (dt) * 1.e-6;	// in s
		t0 = t;
		
		// read imu measurements
		imu.read_accel_gyro_rps(ax, ay, az, gx_rps, gy_rps, gz_rps);
		imu.read_mag(mx, my, mz);
		
		// TODO: Check if there is a benefit from magnetometer data. For GPS it is required however.
		madgwickFilter.get_euler_quaternion(dt_s, ax, ay, az, gx_rps, gy_rps, gz_rps, 0, 0, 0, roll_angle, pitch_angle, yaw_angle, pose_q);
		
		accelAngles(roll_angle_accel, pitch_angle_accel);
		
		// pose is estimated if filtered x- and y-axis angles, as well as z-axis angular velocity, converged
		if ((abs(roll_angle_accel - roll_angle) < init_angleDifference) && (abs(pitch_angle_accel - pitch_angle) < init_angleDifference)
		&& ((abs(yaw_angle - yaw_angle_init) / dt_s) < init_rate)) {
			// reduce beta value, since filtered angles have stabilized during initialisation
			madgwickFilter.set_beta(beta);
			
			DEBUG_PRINTLN(F("Initial pose estimated."));
			DEBUG_PRINTLN2(abs(roll_angle_accel - roll_angle), 6);
			DEBUG_PRINTLN2(abs(pitch_angle_accel - pitch_angle), 6);
			DEBUG_PRINTLN2(abs(yaw_angle - yaw_angle_init) / dt_s, 6);
			
			break;
		}
		yaw_angle_init = yaw_angle;
		
		// run serial print at a rate independent of the main loop
		static uint32_t t0_serial = micros();
		if (micros() - t0_serial > 16666) {
			t0_serial = micros();
			
			DEBUG_PRINT(roll_angle_accel); DEBUG_PRINT("\t"); DEBUG_PRINTLN(pitch_angle_accel);
			DEBUG_PRINT(roll_angle); DEBUG_PRINT("\t"); DEBUG_PRINT(pitch_angle); DEBUG_PRINT("\t"); DEBUG_PRINTLN(yaw_angle);
		}
	}
}

// calculate accelerometer x and y angles in degrees
void accelAngles(float& roll_angle_accel, float& pitch_angle_accel) {
	roll_angle_accel = atan2(ay, az) * RAD2DEG;
	pitch_angle_accel = atan2(-ax, sqrt(pow(ay, 2) + pow(az, 2))) * RAD2DEG;
}

// estimate initial altitude
void estimateAltitude(float init_velocity_v) {
	DEBUG_PRINTLN(F("Estimating initial altitude. Keep device at rest ..."));
	while (1) {
		while (!imuInterrupt) {
			// wait for next imu interrupt
		}
		// reset imu interrupt flag
		imuInterrupt = false;
		
		// blink LED fast to indicate altitude estimation
		updateLED(LED_PIN, 1, 500);
		
		// update time
		t = micros();
		dt = (t - t0);	// in us
		dt_s = (float) (dt) * 1.e-6;	// in s
		t0 = t;
		
		// read imu measurements
		imu.read_accel_gyro_rps(ax, ay, az, gx_rps, gy_rps, gz_rps);
		imu.read_mag(mx, my, mz);
		
		// TODO: Check if magnetometer data from imu is reliable if motors are running
		madgwickFilter.get_euler_quaternion(dt_s, ax, ay, az, gx_rps, gy_rps, gz_rps, 0, 0, 0, roll_angle, pitch_angle, yaw_angle, pose_q);
		
		// calculate ned-acceleration relative to gravity in m/s²
		float a_n_rel, a_e_rel, a_d_rel;
		calc_accel_ned_rel(a_n_rel, a_e_rel, a_d_rel);
		
		// get Kalman filtered altitude in m
		altitudeFilter.update(a_d_rel, baroAltitude, dt_s);
		altitude = altitudeFilter.get_position();
		
		// TODO: Use velocity from altitudeFilter to check if velocity converged. Therefor it has to be below a defined limit for a defined amount of time.
		static uint32_t dt_baro;
		if (barometerInterrupt) {
			barometerInterrupt = false;
			bmp388.getAltitude(baroAltitude);
			
			if (dt_baro > 1000000) {
				DEBUG_PRINTLN(altitude);
				
				// altitude is estimated if vertical velocity converged
				if ((abs(altitude - altitude_init) * 1000000 / dt_baro) < init_velocity_v) {
					DEBUG_PRINTLN(F("Initial altitude estimated."));
					DEBUG_PRINTLN2(abs(altitude - altitude_init) * 1000000 / dt_baro, 2);
					
					break;
				}
				altitude_init = altitude;
				dt_baro = 0;
			}
		}
		dt_baro += dt;
	}
}

// calculate acceleration in ned-frame relative to gravity using acceleration in sensor-frame and pose
void calc_accel_ned_rel(float &a_n_rel, float &a_e_rel, float &a_d_rel) {
	// acceleration of gravity is m/s²
	static const float G = 9.81;
	
	// acceleration quaternion in sensor-frame
	static float a_q[4];
	a_q[0] = 0; a_q[1] = ax; a_q[2] = ay; a_q[3] = az;
	// acceleration quaternion in ned-frame
	static float a_ned_q[4];
	
	// conjugation of pose_q, which is equal to the inverse of pose_q, since it is a unit quaternion
	static float pose_q_conj[4];
	pose_q_conj[0] = pose_q[0]; pose_q_conj[1] = -pose_q[1]; pose_q_conj[2] = -pose_q[2]; pose_q_conj[3] = -pose_q[3];
	// helper quaternion to save some result
	static float helper_q[4];
	
	// calculate the acceleration in ned-frame by rotating the accelerometer vector (sensor frame) with the pose quaternion (unit quaternion): pose_q a_q pose_q_conj 
	qMultiply(pose_q, a_q, helper_q);
	qMultiply(helper_q, pose_q_conj, a_ned_q);
	
	// get ned-acceleration relative to gravity in m/s²
	a_n_rel = a_ned_q[1] * accelRes * G;
	a_e_rel = a_ned_q[2] * accelRes * G;
	a_d_rel = (a_ned_q[3] * accelRes - 1) * G;
}

// multiply two quaternions (Hamilton product)
void qMultiply(float* q1, float* q2, float* result_q) {
	result_q[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
	result_q[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
	result_q[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
	result_q[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
}

// arm/disarm on rc command or disarm on failsafe conditions
void arm_failsafe(uint8_t fs_config) {
	static uint32_t t_arm_failsafe;
	t_arm_failsafe = micros();
	
	// -------------------- auto disarm
	// TODO
	
	// -------------------- arm and disarm on rc command
	static uint32_t t_arm = 0, t_disarm = 0;
	if (rc_channelValue[ARM] == 2000) {	// arm switch needs to be set to enable arming, else disarm and reset
		if ((rc_channelValue[THROTTLE] < 1100) && (((rc_channelValue[ROLL] > 1400) && (rc_channelValue[ROLL] < 1600)) && ((rc_channelValue[PITCH] > 1400) && (rc_channelValue[PITCH] < 1600)))) {
			if ((rc_channelValue[YAW] > 1900)  && (motors.getState() == MotorsQuad::State::disarmed)) {
				// hold left stick bottom-right and keep right stick centered (2s) to complete arming
				t_disarm = 0;
				if (t_arm == 0) {
					t_arm = t_arm_failsafe;
				}
				else if ((t_arm_failsafe - t_arm) > 2000000) {
					motors.arm();
					t_arm = 0;
				}
			}
			else if ((rc_channelValue[YAW] < 1100) && ((motors.getState() == MotorsQuad::State::armed) || (motors.getState() == MotorsQuad::State::arming))) {
				// hold left stick bottom-left and keep right stick centered (2s) to complete disarming
				t_arm = 0;
				if (t_disarm == 0) {
					t_disarm = t_arm_failsafe;
				}
				else if ((t_arm_failsafe - t_disarm) > 2000000) {
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
				error_code = error_code | ERROR_IMU;
				DEBUG_PRINTLN("IMU failsafe caused by major IMU error!");
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
				else if ((t_arm_failsafe - t_fs_motion) > FS_TIME) {
					disarmAndResetQuad();
					t_fs_motion = 0;
					DEBUG_PRINTLN("Motion failsafe!");
				}
			}
			else {
				t_fs_motion = 0;
			}
		}
		else {
			t_fs_motion = 0;
		}
		
		// failsafes for when quadcopter has started
		if (started) {
			// quadcopter control failsafe
			static uint32_t t_fs_control = 0;
			if ((FS_CONFIG & FS_CONTROL) == FS_CONTROL) {
				if ((abs(roll_angle_sp - roll_angle) > FS_CONTROL_ANGLE_DIFF) || (abs(pitch_angle_sp - pitch_angle) > FS_CONTROL_ANGLE_DIFF) || (abs(yaw_rate_sp - yaw_rate) > FS_CONTROL_RATE_DIFF)) {
					// difference to control values for angle and angular rate exceeded
					if (t_fs_control == 0) {
						t_fs_control = t_arm_failsafe;
					}
					else if ((t_arm_failsafe - t_fs_control) > FS_TIME) {
						disarmAndResetQuad();
						t_fs_control = 0;
						DEBUG_PRINTLN("Control failsafe!");
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
		
		// TODO: add additional failsafes here
		
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
		
		// reset PID controller
		roll_rate_pid.reset();
		pitch_rate_pid.reset();
		yaw_rate_pid.reset();
		velocity_v_pid.reset();
		
		roll_rate_mv = 0;
		pitch_rate_mv = 0;
		yaw_rate_mv = 0;
		velocity_v_mv = 0;
		
		// disarm motors
		motors.disarm();
		
		// set started state to false - minimum throttle is required again to start the motors and PID calculations
		started = false;
}
