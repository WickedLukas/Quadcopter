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
// TODO: Uncomment and test LED notification

// print debug outputs through serial
#define DEBUG

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
#define ADDRESS_EEPROM 0

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

#define BETA_INIT 10	// Madgwick algorithm gain (2 * proportional gain (Kp)) during initial pose estimation
#define BETA 0.041		// Madgwick algorithm gain (2 * proportional gain (Kp))

// parameters to check if filtered angles converged during initialisation
#define INIT_ANGLE_DIFFERENCE 0.5		// Maximum angle difference between filtered angles and accelerometer angles (x- and y-axis) after initialisation
#define INIT_ANGULAR_VELOCITY 0.05	// Maximum angular velocity of filtered angle (z-axis) after initialisation

// limits for flight setpoints
#define ROLL_LIMIT		30		// deg
#define PITCH_LIMIT		30		// deg
#define YAW_LIMIT		180			// deg/s
#define THROTTLE_LIMIT	1750	// < 2000 because there is some headroom needed for pid control, so the quadcopter stays stable during full throttle

// moving average filter configuration for the flight setpoints
// TODO: optimize these filter values
#define EMA_ROLL_SP				0.0002
#define EMA_PITCH_SP			0.0002
#define EMA_THROTTLE_SP		0.0002
#define EMA_YAW_VELOCITY_SP		0.0010

// moving average filter configuration for the yaw velocity
#define EMA_YAW_VELOCITY			0.0020	// TODO: optimize this filter value

//TODO: Check if it is better to use a cascaded PID loop controlling rotational rate and angle
// PID values for pose controller
const float P_roll = 1,		I_roll = 0,		D_roll = 0;
const float P_pitch = 1,	I_pitch = 0,	D_pitch = 0;
const float P_yaw = 1,		I_yaw = 0,		D_yaw = 0;
//const float P_level = 1,	I_level = 0,	D_level = 0;

// Minimum throttle setpoint to enter started state in which motors and PID calculation start.
// To ensure a smooth start this value should be close to the throttle necessary for take off.
const float MIN_THROTTLE_SP = 1300;

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
#define FS_MOTION_ANGLE_LIMIT 40
#define FS_MOTION_ANGULAR_VELOCITY_LIMIT 240

// failsafe control limits
#define FS_CONTROL_ANGLE_DIFF 25
#define FS_CONTROL_ANGULAR_VELOCITY_DIFF 150

// list of error codes
const uint8_t ERROR_MAG = 0b00000001;
// Stores the errors which occured and disables arming.
// The flight controller needs to be restarted to reset the error and enable arming again.
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
float angle_z_init = 1000;

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

// pose PID controller
PID_controller roll_pid(P_roll, I_roll, D_roll, 0, 0, 2000);
PID_controller pitch_pid(P_pitch, I_pitch, D_yaw, 0, 0, 2000);
PID_controller yaw_pid(P_yaw, I_yaw, D_yaw, 0, 0, 2000);

// pointer on an array of 10 received rc channel values [1000; 2000]
uint16_t *rc_channelValue;

// flight setpoints
float roll_sp, pitch_sp, yaw_velocity_sp, throttle_sp;

// variables to measure imu update time
uint32_t t0 = 0, t = 0;
// measured imu update time in microseconds
int32_t dt = 0;
// measured imu update time in seconds
float dt_s = 0;

// imu measurements
int16_t ax, ay, az;
float gx_rps, gy_rps, gz_rps;
int16_t mx = 0, my = 0, mz = 0;

// quadcopter pose
float angle_x, angle_y, angle_z;

// z-axis pose offset to compensate for the sensor mounting orientation relative to the quadcopter frame
float angle_z_offset = 90;

// quadcopter z-axis (yaw) velocity
float angular_velocity_z;

// armed state - motors can only run when armed
bool armed = false;

// started state is set when minimum throttle setpoint was reached in armed state - motors and PID calculations can only run when started
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
void estimatePose(float beta_init, float beta, float init_angleDifference, float init_angularVelocity);

// calculate accelerometer x and y angles in degrees
void accelAngles(float& angle_x_accel, float& angle_y_accel);

// calibrate gyroscope, accelerometer or magnetometer on rc command and return true if any calibration was performed
bool imuCalibration();

// disarm and reset quadcopter
void disarmAndResetQuad();

// arm/disarm on rc command and disarm on failsafe conditions
void arm_failsafe(uint8_t fs_config);

// calculate flight setpoints from rc input for stable flightmode without and with tilt compensated thrust
void flightSetpoints(float& roll_sp, float& pitch_sp, float& yaw_velocity_sp, float& throttle_sp);

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
}

void loop() {
	if (error_code != 0) {
		// blink LED very fast to indicate the error occurence
		updateLED(LED_PIN, 2, 250);
	}
	else if (armed) {
		// blink LED normally to indicate armed status
		updateLED(LED_PIN, 2);
	}
	else {
		// turn off LED to indicate disarmed status
		updateLED(LED_PIN, 0);
	}
	
	// update rc
	rc.update();
	
	static bool initialise = false;
	if (!initialise) {
		// get configuration data from EEPROM
		EEPROM.get(ADDRESS_EEPROM, data_eeprom);
		
		// initialise imu
		initialise = imu.init(data_eeprom.offset_mx, data_eeprom.offset_my, data_eeprom.offset_mz, data_eeprom.scale_mx, data_eeprom.scale_my, data_eeprom.scale_mz);
		if (!initialise) {
			DEBUG_PRINTLN(F("IMU initialisation failed."));
		}
		
		// estimate initial pose
		estimatePose(BETA_INIT, BETA, INIT_ANGLE_DIFFERENCE, INIT_ANGULAR_VELOCITY);
	}

	// if disarmed, check for calibration request from rc and executed it
	if (imuCalibration()) {
		// after calibration was performed, imu needs to be initalised again
		initialise = false;
		return;
	}

	// arm/disarm on rc command or disarm on failsafe conditions
	arm_failsafe(FS_CONFIG);

	// update time
	t = micros();
	dt = (t - t0);  // in us
	dt_s = (float) (dt) * 1.e-6;	// in s
	
	while (!imuInterrupt) {
		// wait for next imu interrupt
	}
	
	// reset imu interrupt flag
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
	
	madgwickFilter.get_euler(dt_s, ax, ay, az, gx_rps, gy_rps, gz_rps, mx, my, mz, angle_x, angle_y, angle_z);

	// calculate filtered z-angle (yaw) velocity, since it is used as a control variable instead of the already filtered angle
	static float angle_z0;
	angular_velocity_z = ema_filter((angle_z - angle_z0) / dt_s, angular_velocity_z, EMA_YAW_VELOCITY);
	angle_z0 = angle_z;
	
	// apply offset to z-axis pose in order to compensate for the sensor mounting orientation relative to the quadcopter frame
	angle_z += angle_z_offset;
	if (angle_z > 180) {
		angle_z -= 360;
	}
	else if (angle_z < -180) {
		angle_z += 360;
	}
	
	// when armed, calculate flight setpoints, manipulated variables and control motors
	if (armed) {
		// calculate flight setpoints
		flightSetpoints(roll_sp, pitch_sp, yaw_velocity_sp, throttle_sp);
		
		// In order to ensure a smooth start, delay running motors as well as PID calculations until a minimum throttle value is applied
		if (started) {
			// get manipulated variables
			static float roll_mv, pitch_mv, yaw_mv;
			roll_mv = roll_pid.get_mv(roll_sp, angle_x, dt_s);
			pitch_mv = pitch_pid.get_mv(pitch_sp, angle_y, dt_s);
			yaw_mv = yaw_pid.get_mv(yaw_velocity_sp, angular_velocity_z, dt_s);
			
			// TODO: Check motor mixing logic carefully
			/*motor_1.write(throttle_sp + roll_mv - pitch_mv + yaw_mv);
			motor_2.write(throttle_sp - roll_mv - pitch_mv - yaw_mv);
			motor_3.write(throttle_sp + roll_mv + pitch_mv + yaw_mv);
			motor_4.write(throttle_sp - roll_mv + pitch_mv - yaw_mv);*/

			// TODO: Remove this test code
			/*motor_1.write(rc_channelValue[THROTTLE]);
			motor_2.write(rc_channelValue[THROTTLE]);
			motor_3.write(rc_channelValue[THROTTLE]);
			motor_4.write(rc_channelValue[THROTTLE]);*/
		}
		else{
			if (throttle_sp > MIN_THROTTLE_SP) {
				started = true;
				DEBUG_PRINTLN("Started!");
			}
		}
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
		
		//static float angle_x_accel, angle_y_accel;
		//accelAngles(angle_x_accel, angle_y_accel);
		//DEBUG_PRINT(angle_x_accel); DEBUG_PRINT("\t"); DEBUG_PRINTLN(angle_y_accel);
		//DEBUG_PRINT(angle_x); DEBUG_PRINT("\t"); DEBUG_PRINT(angle_y); DEBUG_PRINT("\t"); DEBUG_PRINTLN(angle_z);
		
		//DEBUG_PRINTLN(dt);
		//DEBUG_PRINTLN();
		
		// print channel values
		/*for (int i=0; i<10 ; i++) {
			DEBUG_PRINT(rc_channelValue[i]); DEBUG_PRINT("\t");
		}
		DEBUG_PRINTLN();*/
		
		#ifdef SEND_SERIAL
			// Send data to "Processing" for visualization
			sendSerial(dt, angle_x, angle_y, angle_z);
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
void estimatePose(float beta_init, float beta, float init_angleDifference, float init_angularVelocity) {
	// angles calculated from accelerometer
	float angle_x_accel, angle_y_accel;

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
		
		madgwickFilter.get_euler(dt_s, ax, ay, az, gx_rps, gy_rps, gz_rps, mx, my, mz, angle_x, angle_y, angle_z);
		
		accelAngles(angle_x_accel, angle_y_accel);
		
		// pose is estimated if filtered x- and y-axis angles, as well as z-axis angular velocity, converged
		if ((abs(angle_x_accel - angle_x) < init_angleDifference) && (abs(angle_y_accel - angle_y) < init_angleDifference)
		&& ((abs(angle_z - angle_z_init) / dt_s) < init_angularVelocity)) {
			// reduce beta value, since filtered angles have stabilized during initialisation
			madgwickFilter.set_beta(beta);
			
			DEBUG_PRINTLN(F("Initial pose estimated."));
			DEBUG_PRINTLN2(abs(angle_x_accel - angle_x), 6);
			DEBUG_PRINTLN2(abs(angle_y_accel - angle_y), 6);
			DEBUG_PRINTLN2(abs(angle_z - angle_z_init) / dt_s, 6);
			
			break;
		}
		angle_z_init = angle_z;
		
		// run serial print at a rate independent of the main loop
		static uint32_t t0_serial = micros();
		if (micros() - t0_serial > 16666) {
			t0_serial = micros();
			
			DEBUG_PRINT(angle_x_accel); DEBUG_PRINT("\t"); DEBUG_PRINTLN(angle_y_accel);
			DEBUG_PRINT(angle_x); DEBUG_PRINT("\t"); DEBUG_PRINT(angle_y); DEBUG_PRINT("\t"); DEBUG_PRINTLN(angle_z);
		}
	}
}

// calculate accelerometer x and y angles in degrees
void accelAngles(float& angle_x_accel, float& angle_y_accel) {
	static const float RAD2DEG = 4068 / 71;
	
	angle_x_accel = atan2(ay, az) * RAD2DEG;
	angle_y_accel = atan2(-ax, sqrt(pow(ay,2) + pow(az,2))) * RAD2DEG;
}

// calibrate gyroscope, accelerometer or magnetometer on rc command and return true if any calibration was performed
bool imuCalibration() {
	static uint32_t t_calibrateGyro;
	static uint32_t t_calibrateAccel;
	static uint32_t t_calibrateMag;
	
	if (!armed) {
		if ((rc_channelValue[PITCH] < 1050) && (rc_channelValue[ROLL] > 1450) && (rc_channelValue[ROLL] < 1550)) {
			if ((rc_channelValue[THROTTLE] < 1050) && (rc_channelValue[YAW] < 1050)) {
				// hold right stick bottom center and left stick bottom-left to start gyro calibration	(2s)
				t_calibrateGyro += dt;
				t_calibrateAccel = 0;
				t_calibrateMag = 0;
				
				if (t_calibrateGyro > 2000000) {
					// turn on LED to indicate calibration
					updateLED(LED_PIN, 2);
					imu.calibrate_gyro(imuInterrupt, 5.0, 1);
					t_calibrateGyro = 0;
					return true;
				}
				else {
					return false;
				}
			}
			else if ((rc_channelValue[THROTTLE] > 1950) && (rc_channelValue[YAW] < 1050)) {
				// hold right stick bottom center and left stick top-left to start accel calibration	(2s)
				t_calibrateGyro = 0;
				t_calibrateAccel += dt;
				t_calibrateMag = 0;
				
				if (t_calibrateAccel > 2000000) {
					// turn on LED to indicate calibration
					updateLED(LED_PIN, 2);
					imu.calibrate_accel(imuInterrupt, 5.0, 16);
					t_calibrateAccel = 0;
					return true;
				}
				else {
					return false;
				}
			}
			else if ((rc_channelValue[THROTTLE] > 1950) && (rc_channelValue[YAW] > 1950)) {
				// hold right stick bottom center and left stick top-right to start mag calibration	(2s)
				t_calibrateGyro = 0;
				t_calibrateAccel = 0;
				t_calibrateMag += dt;
				
				if (t_calibrateMag > 2000000) {
					// turn on LED to indicate calibration
					updateLED(LED_PIN, 2);
					imu.calibrate_mag(imuInterrupt, 60, 0, data_eeprom.offset_mx, data_eeprom.offset_my, data_eeprom.offset_mz, data_eeprom.scale_mx, data_eeprom.scale_my, data_eeprom.scale_mz);
					EEPROM.put(ADDRESS_EEPROM, data_eeprom);
					t_calibrateMag = 0;
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
	
	// turn off LED
	updateLED(LED_PIN, 0);
	
	return false;
}

// disarm and reset quadcopter
void disarmAndResetQuad() {
		// set flight setpoints to zero
		roll_sp = 0;
		pitch_sp = 0;
		yaw_velocity_sp = 0;
		throttle_sp = 0;
		
		// reset PIDs
		roll_pid.reset();
		pitch_pid.reset();
		yaw_pid.reset();
		
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
	// ---------- arm and disarm on rc command ----------
	static uint32_t t_arm;
	static uint32_t t_disarm;
	if (rc_channelValue[ARM] == 2000) {		// arm switch needs to be set to enable arming, else disarm and reset
		if (rc_channelValue[THROTTLE] < 1050) {
			if ((rc_channelValue[YAW] > 1950) && (error_code == 0)) {	// arming is only allowed when no error occured
			  // hold left stick bottom-right	(2s) to complete arming
				t_arm += dt;
				t_disarm = 0;
				
				if (t_arm > 2000000) {
					armed = true;
					t_arm = 0;
					DEBUG_PRINTLN("Armed!");
				}
			}
			else if (rc_channelValue[YAW] < 1050) {
				// hold left stick bottom-left (2s) to complete disarming
				t_arm = 0;
				t_disarm += dt;
				
				if (t_disarm > 2000000) {
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
	else {
		disarmAndResetQuad();
		t_arm = 0;
		t_disarm = 0;
	}
	
	// ---------- Disarm on failsafe conditions ----------
	static uint32_t t_fs_motion;
	static uint32_t t_fs_control;
	
	// imu failsafe
	if ((FS_CONFIG & FS_IMU) == FS_IMU) {
		if (dt > FS_IMU_DT_LIMIT) {
			// limit for imu update time exceeded
			disarmAndResetQuad();
			DEBUG_PRINTLN("IMU failsafe!");
		}
	}
	
	// quadcopter motion failsafe
	if ((FS_CONFIG & FS_MOTION) == FS_MOTION) {
		if ((abs(angle_x) > FS_MOTION_ANGLE_LIMIT) || (abs(angle_y) > FS_MOTION_ANGLE_LIMIT) || (abs(angular_velocity_z) > FS_MOTION_ANGULAR_VELOCITY_LIMIT)) {
			// angle or angular velocity limit exceeded
			t_fs_motion += dt;
			if (t_fs_motion >= FS_TIME) {
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
		if ((FS_CONFIG & FS_CONTROL) == FS_CONTROL) {
			if ((abs(roll_sp - angle_x) > FS_CONTROL_ANGLE_DIFF) || (abs(pitch_sp - angle_y) > FS_CONTROL_ANGLE_DIFF) || (abs(yaw_velocity_sp - angular_velocity_z) > FS_CONTROL_ANGULAR_VELOCITY_DIFF)) {
				// difference to control values for angle and angular velocity exceeded
				t_fs_control += dt;
				if (t_fs_control >= FS_TIME) {
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
	
	// TODO: Add additional failsafe conditions
}

// calculate flight setpoints from rc input for stable flightmode without and with tilt compensated thrust
void flightSetpoints(float& roll_sp, float& pitch_sp, float& yaw_velocity_sp, float& throttle_sp) {
	static const float DEG2RAD = 71 / 4068;
	static float roll_mapped, pitch_mapped, yaw_mapped, throttle_mapped;
	
	// map rc channel values
	roll_mapped = map((float) rc_channelValue[ROLL], 1000, 2000, -ROLL_LIMIT, ROLL_LIMIT);
	pitch_mapped = map((float) rc_channelValue[PITCH], 1000, 2000, -PITCH_LIMIT, PITCH_LIMIT);
	yaw_mapped = map((float) rc_channelValue[YAW], 1000, 2000, -YAW_LIMIT, YAW_LIMIT);

	if (rc_channelValue[FMODE] == 1500) {
		// stable with tilt compensated thrust: increase thrust when quadcopter is tilted, to compensate for height loss during horizontal movement
		throttle_mapped = map((float) rc_channelValue[THROTTLE] / (cos(angle_x * DEG2RAD) * cos(angle_y * DEG2RAD)), 1000, 2000, 1000, THROTTLE_LIMIT);
	}
	else {
		// stable
		throttle_mapped = map((float) rc_channelValue[THROTTLE], 1000, 2000, 1000, THROTTLE_LIMIT);
	}
	
	// calculate flight setpoints by filtering the mapped rc channel values
	roll_sp = ema_filter(roll_mapped, roll_sp, EMA_ROLL_SP);
	pitch_sp = ema_filter(pitch_mapped, pitch_sp, EMA_PITCH_SP);
	yaw_velocity_sp = ema_filter(yaw_mapped, roll_sp, EMA_YAW_VELOCITY_SP);
	throttle_sp = ema_filter(throttle_mapped, throttle_sp, EMA_THROTTLE_SP);
}
