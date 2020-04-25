/*
* Quadcopter.ino
*
* Created:	26.04.2019
* Author:	Lukas
*
*/
#include <Arduino.h>
#include <EEPROM.h>

#include "iBus.h"
#include "ICM20948.h"
#include "MadgwickAHRS.h"
#include "ema_filter.h"
#include "PID_controller.h"

#include "sendSerial.h"

// TODO: integrate telemetry (MAVLink?)

// print debug outputs through serial
//#define DEBUG

// send imu data through serial (for example to visualize it in "Processing")
//#define SEND_SERIAL

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

// imu pins
#define IMU_SPI_PORT SPI
#define IMU_CS_PIN 10
#define IMU_INTERRUPT_PIN 9

// pwm pins to control motors
// 1: top-left (CW); 2: top-right (CCW); 3: bottom-left (CW); 4: bottom-right (CCW);
#define MOTOR_PIN_1 2
#define MOTOR_PIN_2 3
#define MOTOR_PIN_3 4
#define MOTOR_PIN_4 5

// motor pwm resolution
#define MOTOR_PWM_RESOLUTION 11

// motor pwm frequency
#define MOTOR_PWM_FREQENCY 12000

// rc channel assignment
#define ROLL			0
#define PITCH			1
#define YAW				3
#define THROTTLE	2
#define ARM				6	// disarmed: 1000, armed: 2000
#define FMODE			8	// stable: 1000/2000, stable with tilt compensated thrust: 1500, acro (not implemented)

#define BETA_INIT 10		// Madgwick algorithm gain (2 * proportional gain (Kp)) during initial pose estimation
#define BETA 			0.041	// Madgwick algorithm gain (2 * proportional gain (Kp))

// parameters to check if filtered angles converged during initialisation
#define INIT_ANGLE_DIFFERENCE 0.5		// maximum angle difference between filtered angles and accelerometer angles (x- and y-axis) after initialisation
#define INIT_RATE 0.05	// maximum angular rate of Madgwick-filtered angle (z-axis) after initialisation

// limits for the flight setpoints
#define ROLL_RATE_LIMIT		180		// deg/s
#define PITCH_RATE_LIMIT	180		// deg/s
#define YAW_RATE_LIMIT		180		// deg/s

#define ROLL_ANGLE_LIMIT	45		// deg
#define PITCH_ANGLE_LIMIT	45		// deg

#define THROTTLE_LIMIT		1700	// < 2000 because there is some headroom needed for pid control, so the quadcopter stays stable during full throttle

// moving average filter configuration for the flight setpoints
#define EMA_ROLL_ANGLE_SP		0.0007
#define EMA_PITCH_ANGLE_SP	0.0007
#define EMA_THROTTLE_SP			0.0007
#define EMA_YAW_RATE_SP			0.0007

// moving average filter configuration for the angular rates (gyro)
// TODO: Maybe unnecessary or use notch filter instead
//#define EMA_ROLL_RATE	0.0020
//#define EMA_PITCH_RATE	0.0020
//#define EMA_YAW_RATE	0.0020

// PID values for the angular rate controller (inner loop)
const float P_ROLL_RATE = 1,	I_ROLL_RATE = 0,	D_ROLL_RATE = 0;
const float P_PITCH_RATE = 1,	I_PITCH_RATE = 0,	D_PITCH_RATE = 0;
const float P_YAW_RATE = 1,		I_YAW_RATE = 0,		D_YAW_RATE = 0;

// PID values for the angle controller (outer loop)
const float P_ROLL_ANGLE = 1,		I_ROLL_ANGLE = 0,		D_ROLL_ANGLE = 0;
const float P_PITCH_ANGLE = 1,	I_PITCH_ANGLE = 0,	D_PITCH_ANGLE = 0;
const float P_YAW_ANGLE = 1,		I_YAW_ANGLE = 0,		D_YAW_ANGLE = 0;

// Factors for forwarding setpoint angles into the angular rate controller. Higher values improve the quadcopter response to inputs.
const float FF_ROLL = 0;
const float FF_PITCH = 0;

// Minimum throttle setpoint to enter started state in which PID calculation start.
// To ensure a smooth start this value should be close to the throttle necessary for take off.
const float MIN_THROTTLE_SP = 1200;

// failsafe configuration
const uint8_t FS_IMU			= 0b00000001;
const uint8_t FS_MOTION		= 0b00000010;
const uint8_t FS_CONTROL	= 0b00000100;

// use imu and motion failsafe
const uint8_t FS_CONFIG		= 0b00000011;

// minimum time in microseconds a failsafe condition needs to be met, in order to activate it (except for FS_IMU)
#define FS_TIME 500000

// failsafe imu update time limit
#define FS_IMU_DT_LIMIT 2000000

// failsafe motion limits
#define FS_MOTION_ANGLE_LIMIT 60
#define FS_MOTION_RATE_LIMIT 270

// failsafe control limits
#define FS_CONTROL_ANGLE_DIFF 30
#define FS_CONTROL_RATE_DIFF 120

// factor for converting a radian number to an equivalent number in degrees
const float RAD2DEG = (float) 4068 / 71;

// list of error codes
const uint8_t ERROR_IMU = 0b00000001;
const uint8_t ERROR_MAG = 0b00000010;
// Stores the errors which occured and disables arming.
// A restart is required to reset the error and enable arming.
uint8_t error_code = 0;

// configuration data structure
typedef struct {
	// magnetometer hard iron distortion correction
	float offset_mx, offset_my, offset_mz;
	// magnetometer soft iron distortion correction
	float scale_mx, scale_my, scale_mz;
} config_data;

// configuration data
config_data data_eeprom;

// radio control (rc) object
IBUS rc;

// ICM-20948 imu object
ICM20948_SPI imu(IMU_CS_PIN, IMU_SPI_PORT);
	
// initial quadcopter z-axis angle - this should be initialised with an implausible value (> 360)
float yaw_angle_init = 1000;

// motor class
class PWMServoMotor
{
	public:
	
	PWMServoMotor(uint8_t pin, uint8_t motor_pwm_resolution, uint16_t motor_pwm_frequency) {
		m_pin = pin;
		m_motor_pwm_resolution = motor_pwm_resolution;
		
		analogWriteFrequency(pin, motor_pwm_frequency);
		digitalWrite(pin, LOW);
		pinMode(pin, OUTPUT);
	}
	
	void write(uint16_t value) {
		noInterrupts();
		m_oldRes = analogWriteResolution(m_motor_pwm_resolution);
		analogWrite(m_pin, value);
		analogWriteResolution(m_oldRes);
		interrupts();
	}
	
	private:
	
	uint8_t m_pin;
	uint8_t m_motor_pwm_resolution;
	static uint8_t m_oldRes;
};

uint8_t PWMServoMotor::m_oldRes;

// motor objects
PWMServoMotor motor_1(MOTOR_PIN_1, MOTOR_PWM_RESOLUTION, MOTOR_PWM_FREQENCY);
PWMServoMotor motor_2(MOTOR_PIN_2, MOTOR_PWM_RESOLUTION, MOTOR_PWM_FREQENCY);
PWMServoMotor motor_3(MOTOR_PIN_3, MOTOR_PWM_RESOLUTION, MOTOR_PWM_FREQENCY);
PWMServoMotor motor_4(MOTOR_PIN_4, MOTOR_PWM_RESOLUTION, MOTOR_PWM_FREQENCY);

// Madgwick filter object
MADGWICK_AHRS madgwickFilter(BETA_INIT);

// rate PID controller (inner loop)
PID_controller roll_rate_pid(P_ROLL_RATE, I_ROLL_RATE, D_ROLL_RATE, 0, 0, ROLL_RATE_LIMIT);
PID_controller pitch_rate_pid(P_PITCH_RATE, I_PITCH_RATE, D_YAW_RATE, 0, 0, PITCH_RATE_LIMIT);
PID_controller yaw_rate_pid(P_YAW_RATE, I_YAW_RATE, D_YAW_RATE, 0, 0, YAW_RATE_LIMIT);

// angle PID controller (outer loop)
PID_controller roll_angle_pid(P_ROLL_ANGLE, I_ROLL_ANGLE, D_ROLL_ANGLE, 0, 0, ROLL_ANGLE_LIMIT);
PID_controller pitch_angle_pid(P_PITCH_ANGLE, I_PITCH_ANGLE, D_YAW_ANGLE, 0, 0, PITCH_ANGLE_LIMIT);

// variables to measure imu update time
uint32_t t0 = 0, t = 0;
// measured imu update time in microseconds
int32_t dt = 0;
// measured imu update time in seconds
float dt_s = 0;

// pointer on an array of 10 received rc channel values [1000; 2000]
uint16_t *rc_channelValue;

// flight setpoints
float roll_angle_sp, pitch_angle_sp, yaw_rate_sp, throttle_sp;

// imu measurements
int16_t ax, ay, az;
float gx_rps, gy_rps, gz_rps;
int16_t mx = 0, my = 0, mz = 0;

// quadcopter pose
float roll_angle, pitch_angle, yaw_angle;

// z-axis pose offset to compensate for the sensor mounting orientation relative to the quadcopter frame
float yaw_angle_offset = 90;

// filtered gyro rates
float roll_rate, pitch_rate, yaw_rate;

// armed state - motors can only run when armed
bool armed = false;

// started state is set when minimum throttle setpoint was reached in armed state - PID calculations can only run when started
bool started = false;

// imu interrupt
volatile bool imuInterrupt = false;
void imuReady() {
	imuInterrupt = true;
}

// update LED
// mode: 0		off
// mode: 1		blink
// mode: > 1	on
void updateLED(uint8_t pin, uint8_t mode, uint32_t interval_ms = 1000);

// estimate initial pose
void estimatePose(float beta_init, float beta, float init_angleDifference, float init_rate);

// calculate accelerometer x and y angles in degrees
void accelAngles(float& roll_angle_accel, float& pitch_angle_accel);

// calibrate gyroscope, accelerometer or magnetometer on rc command and return true if any calibration was performed
bool imuCalibration();

// disarm and reset quadcopter
void disarmAndResetQuad();

// arm/disarm on rc command and disarm on failsafe conditions
void arm_failsafe(uint8_t fs_config);

// calculate flight setpoints from rc input for stable flightmode without and with tilt compensated thrust
void flightSetpoints(float& roll_angle_sp, float& pitch_angle_sp, float& yaw_rate_sp, float& throttle_sp);

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
	
	// initialise serial for iBus communication
	Serial2.begin(115200, SERIAL_8N1);
	while (!Serial2);
	
	// initialise rc and return a pointer on the received rc channel values
	rc_channelValue = rc.begin(Serial2);
	
	// initialise SPI for imu communication
	IMU_SPI_PORT.begin();
	
	// setup interrupt pin for imu
	pinMode(IMU_INTERRUPT_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), imuReady, FALLING);
	
	// get configuration data from EEPROM
	EEPROM.get(ADDRESS_EEPROM, data_eeprom);
	
	// initialise imu
	if (!imu.init(data_eeprom.offset_mx, data_eeprom.offset_my, data_eeprom.offset_mz, data_eeprom.scale_mx, data_eeprom.scale_my, data_eeprom.scale_mz)) {
		// IMU could not be initialized. Set error value, which will disable arming.
		error_code = error_code | ERROR_IMU;
		DEBUG_PRINTLN("IMU error: Initialisation failed!");
	}
}

void loop() {
	if (error_code != 0) {
		// blink LED very fast to indicate an error occurrence
		updateLED(LED_PIN, 1, 250);
	}
	else if (armed) {
		// blink LED normally to indicate armed status
		updateLED(LED_PIN, 1, 1000);
	}
	else {
		// turn off LED to indicate disarmed status
		updateLED(LED_PIN, 0);
	}
	
	// update rc
	rc.update();
	
	static bool poseEstimated = false;
	if (!poseEstimated) {
		// estimate initial pose
		estimatePose(BETA_INIT, BETA, INIT_ANGLE_DIFFERENCE, INIT_RATE);
		
		poseEstimated = true;
	}

	// if disarmed, check for calibration request from rc and executed it
	if (imuCalibration()) {
		// calibration was performed, initial pose needs to be estimated again
		poseEstimated = false;
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
		if ((dt_mag > FS_IMU_DT_LIMIT) && (error_code & ERROR_MAG != ERROR_MAG)) {
			// Limit for magnetometer update time exceeded. Set error value, which will disable arming.
			error_code = error_code | ERROR_MAG;
			DEBUG_PRINTLN("Magnetometer error!");
		}
	}
	
	// perform sensor fusion with Madgwick filter to calculate pose
	madgwickFilter.get_euler(dt_s, ax, ay, az, gx_rps, gy_rps, gz_rps, mx, my, mz, roll_angle, pitch_angle, yaw_angle);

	// apply offset to z-axis pose in order to compensate for the sensor mounting orientation relative to the quadcopter frame
	yaw_angle += yaw_angle_offset;
	if (yaw_angle > 180) {
		yaw_angle -= 360;
	}
	else if (yaw_angle < -180) {
		yaw_angle += 360;
	}
	
	// gyro rates
	roll_rate = gx_rps * RAD2DEG;
	pitch_rate = gy_rps * RAD2DEG;
	yaw_rate = gz_rps * RAD2DEG;
	// TODO: Maybe unnecessary or use notch filter instead
	//roll_rate = ema_filter(gx_rps * RAD2DEG, roll_rate, EMA_ROLL_RATE);
	//pitch_rate = ema_filter(gy_rps * RAD2DEG, pitch_rate, EMA_PITCH_RATE);
	//yaw_rate = ema_filter(gz_rps * RAD2DEG, yaw_rate, EMA_YAW_RATE);

	// when armed, calculate flight setpoints, manipulated variables and control motors
	if (armed) {
		// calculate flight setpoints
		flightSetpoints(roll_angle_sp, pitch_angle_sp, yaw_rate_sp, throttle_sp);
		
		// In order to ensure a smooth start, PID calculations are delayed until a minimum throttle value is applied.
		static float roll_rate_mv, pitch_rate_mv, yaw_rate_mv;
		static float roll_angle_mv, pitch_angle_mv;
		if (started) {
			// calculate manipulated variables
			
			// outer PID angle controller
			roll_angle_mv = roll_angle_pid.get_mv(roll_angle_sp - roll_angle, roll_angle_mv, dt_s);
			pitch_angle_mv = pitch_angle_pid.get_mv(pitch_angle_sp - pitch_angle, pitch_angle_mv, dt_s);

			// inner PID rate controller with feed forward element
			roll_rate_mv = roll_rate_pid.get_mv(roll_angle_mv - roll_rate + FF_ROLL * roll_angle_sp, roll_rate_mv, dt_s);
			pitch_rate_mv = pitch_rate_pid.get_mv(pitch_angle_mv - pitch_rate + FF_PITCH * pitch_angle_sp, pitch_rate_mv, dt_s);
			yaw_rate_mv = yaw_rate_pid.get_mv(yaw_rate_sp - yaw_rate, yaw_rate_mv, dt_s);
		}
		else if (throttle_sp > MIN_THROTTLE_SP) {
			started = true;
			DEBUG_PRINTLN("Started!");
		}
		
		// TODO: Check motor mixing logic carefully
		/*motor_1.write(throttle_sp + roll_mv - pitch_mv + yaw_mv);
		motor_2.write(throttle_sp - roll_mv - pitch_mv - yaw_mv);
		motor_3.write(throttle_sp + roll_mv + pitch_mv + yaw_mv);
		motor_4.write(throttle_sp - roll_mv + pitch_mv - yaw_mv);*/
		
		// TODO: Remove this test code
		/*roll_rate_pid.set_K_p(map((float) rc_channelValue[4], 1000, 2000, 0, 5));
		motor_1.write(throttle_sp + roll_mv);
		motor_2.write(0);
		motor_3.write(throttle_sp- roll_mv);
		motor_4.write(0);*/
	}
	else {
		// for safety reasons repeat disarm and reset, even when it was already done
		disarmAndResetQuad();
	}
	
	// run serial print at a rate independent of the main loop (t0_serial = 16666 for 60 Hz update rate)
	static uint32_t t0_serial = micros();
	if (micros() - t0_serial > 16666) {
		t0_serial = micros();
		
		//DEBUG_PRINT(ax); DEBUG_PRINT("\t"); DEBUG_PRINT(ay); DEBUG_PRINT("\t"); DEBUG_PRINTLN(az);
		//DEBUG_PRINT(gx_rps); DEBUG_PRINT("\t"); DEBUG_PRINT(gy_rps); DEBUG_PRINT("\t"); DEBUG_PRINTLN(gz_rps);
		//DEBUG_PRINT(mx); DEBUG_PRINT("\t"); DEBUG_PRINT(my); DEBUG_PRINT("\t"); DEBUG_PRINTLN(mz);
		
		//static float roll_angle_accel, pitch_angle_accel;
		//accelAngles(roll_angle_accel, pitch_angle_accel);
		//DEBUG_PRINT(roll_angle_accel); DEBUG_PRINT("\t"); DEBUG_PRINTLN(pitch_angle_accel);
		//DEBUG_PRINT(roll_angle); DEBUG_PRINT("\t"); DEBUG_PRINT(pitch_angle); DEBUG_PRINT("\t"); DEBUG_PRINTLN(yaw_angle);
		
		//DEBUG_PRINT(map((float) rc_channelValue[ROLL], 1000, 2000, -ROLL_ANGLE_LIMIT, ROLL_ANGLE_LIMIT)); DEBUG_PRINT("\t"); DEBUG_PRINTLN(roll_angle_sp);
		//DEBUG_PRINT(map((float) rc_channelValue[YAW], 1000, 2000, -YAW_RATE_LIMIT, YAW_RATE_LIMIT)); DEBUG_PRINT("\t"); DEBUG_PRINTLN(yaw_rate_sp);
		//DEBUG_PRINT(map((float) rc_channelValue[THROTTLE], 1000, 2000, 1000, THROTTLE_LIMIT)); DEBUG_PRINT("\t"); DEBUG_PRINTLN(throttle_sp);
		//static const float DEG2RAD = (float) 71 / 4068;
		//DEBUG_PRINT(map((float) (rc_channelValue[THROTTLE] - 1000) / (cos(roll_angle * DEG2RAD) * cos(pitch_angle * DEG2RAD)) + 1000, 1000, 2000, 1000, THROTTLE_LIMIT)); DEBUG_PRINT("\t"); DEBUG_PRINTLN(throttle_sp);
		
		//DEBUG_PRINTLN(dt);
		//DEBUG_PRINTLN();
		
		// print channel values
		/*for (int i=0; i<10 ; i++) {
			DEBUG_PRINT(rc_channelValue[i]); DEBUG_PRINT("\t");
		}
		DEBUG_PRINTLN();*/
		
		#ifdef SEND_SERIAL
			// Send data to "Processing" for visualization
			sendSerial(dt, roll_angle, pitch_angle, yaw_angle);
		#endif
	}
}

// update LED
// mode: 0		off
// mode: 1		blink
// mode: > 1	on
void updateLED(uint8_t pin, uint8_t mode, uint32_t interval_ms) {
	static uint8_t ledState = LOW;
	static uint32_t t0_ms = 0, t_ms = 0;
	
	if (mode == 0) {
		ledState = LOW;
	}
	else if (mode == 1) {
		t_ms = millis();
		
		if ((t_ms - t0_ms) > interval_ms) {
		t0_ms = t_ms;
			
			if (ledState == LOW) {
				ledState = HIGH;
			}
			else {
				ledState = LOW;
			}
		}
	}
	else {
		ledState = HIGH;
	}
	
	// set LED state
	digitalWrite(pin, ledState);
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
		
		madgwickFilter.get_euler(dt_s, ax, ay, az, gx_rps, gy_rps, gz_rps, mx, my, mz, roll_angle, pitch_angle, yaw_angle);
		
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
	pitch_angle_accel = atan2(-ax, sqrt(pow(ay,2) + pow(az,2))) * RAD2DEG;
}

// calibrate gyroscope, accelerometer or magnetometer on rc command and return true if any calibration was performed
bool imuCalibration() {
	static uint32_t t_imuCalibration;
	t_imuCalibration = micros();
	
	static uint32_t t_calibrateGyro = 0, t_calibrateAccel = 0, t_calibrateMag = 0;
	if (!armed) {
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
					imu.calibrate_gyro(imuInterrupt, 5.0, 1);
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
					imu.calibrate_accel(imuInterrupt, 5.0, 16);
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
					imu.calibrate_mag(imuInterrupt, 60, 500, data_eeprom.offset_mx, data_eeprom.offset_my, data_eeprom.offset_mz, data_eeprom.scale_mx, data_eeprom.scale_my, data_eeprom.scale_mz);
					EEPROM.put(ADDRESS_EEPROM, data_eeprom);
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

// disarm and reset quadcopter
void disarmAndResetQuad() {
		// set flight setpoints to zero
		roll_angle_sp = 0;
		pitch_angle_sp = 0;
		yaw_rate_sp = 0;
		throttle_sp = 1000;
		
		// reset PID controller
		roll_rate_pid.reset();
		pitch_rate_pid.reset();
		yaw_rate_pid.reset();
		
		roll_angle_pid.reset();
		pitch_angle_pid.reset();
		
		// turn off motors
		motor_1.write(0);
		motor_2.write(0);
		motor_3.write(0);
		motor_4.write(0);

		// set armed state to false - armed state can only change to false here, to make sure the motors are really disarmed in this state
		armed = false;
		// set started state to false - minimum throttle is required again to start the motors and PID calculations
		started = false;
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
			if ((rc_channelValue[YAW] > 1900) && (error_code == 0) && (!armed)) {	// arming is only allowed when no error occured
				// hold left stick bottom-right and keep right stick centered (2s) to complete arming
				t_disarm = 0;
				if (t_arm == 0) {
					t_arm = t_arm_failsafe;
				}
				else if ((t_arm_failsafe - t_arm) > 2000000) {
					armed = true;
					t_arm = 0;
					DEBUG_PRINTLN("Armed!");
				}
			}
			else if ((rc_channelValue[YAW] < 1100) && (armed)) {
				// hold left stick bottom-left and keep right stick centered (2s) to complete disarming
				t_arm = 0;
				if (t_disarm == 0) {
					t_disarm = t_arm_failsafe;
				}
				else if ((t_arm_failsafe - t_disarm) > 2000000) {
					disarmAndResetQuad();
					t_disarm = 0;
					DEBUG_PRINTLN("Disarmed!");
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
	else if (armed) {
		disarmAndResetQuad();
		t_arm = 0;
		t_disarm = 0;
		DEBUG_PRINTLN("Disarmed!");
	}
	else {
		t_arm = 0;
		t_disarm = 0;
	}
	
	// -------------------- Disarm on failsafe conditions
	if (armed) {
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

// calculate flight setpoints from rc input for stable flightmode without and with tilt compensated thrust
void flightSetpoints(float& roll_angle_sp, float& pitch_angle_sp, float& yaw_rate_sp, float& throttle_sp) {
	static const float DEG2RAD = (float) 71 / 4068;
	static float roll_angle_mapped, pitch_angle_mapped, yaw_rate_mapped, throttle_mapped;
	
	// map rc channel values
	roll_angle_mapped = map((float) rc_channelValue[ROLL], 1000, 2000, -ROLL_ANGLE_LIMIT, ROLL_ANGLE_LIMIT);
	pitch_angle_mapped = map((float) rc_channelValue[PITCH], 1000, 2000, -PITCH_ANGLE_LIMIT, PITCH_ANGLE_LIMIT);
	yaw_rate_mapped = map((float) rc_channelValue[YAW], 1000, 2000, -YAW_RATE_LIMIT, YAW_RATE_LIMIT);
	
	if (rc_channelValue[FMODE] == 1500) {
		// stable with tilt compensated thrust: increase thrust when quadcopter is tilted, to compensate for height loss during horizontal movement
		throttle_mapped = map((float) (rc_channelValue[THROTTLE] - 1000) / (cos(roll_angle * DEG2RAD) * cos(pitch_angle * DEG2RAD)) + 1000, 1000, 2000, 1000, THROTTLE_LIMIT);
	}
	else {
		// stable
		throttle_mapped = map((float) rc_channelValue[THROTTLE], 1000, 2000, 1000, THROTTLE_LIMIT);
	}
	
	// calculate flight setpoints by filtering the mapped rc channel values
	roll_angle_sp = ema_filter(roll_angle_mapped, roll_angle_sp, EMA_ROLL_ANGLE_SP);
	pitch_angle_sp = ema_filter(pitch_angle_mapped, pitch_angle_sp, EMA_PITCH_ANGLE_SP);
	yaw_rate_sp = ema_filter(yaw_rate_mapped, yaw_rate_sp, EMA_YAW_RATE_SP);
	throttle_sp = ema_filter(throttle_mapped, throttle_sp, EMA_THROTTLE_SP);
}
