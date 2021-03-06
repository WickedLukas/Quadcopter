#include <Arduino.h>
#include <EEPROM.h>

#include "iBus.h"
#include "ICM20948.h"
#include "BMP388_DEV.h"
#include "MadgwickAHRS.h"
#include "ema_filter.h"
#include "PID_controller.h"
#include "KalmanFilter1D.h"

#include "Plotter.h"
#include "sendSerial.h"

// TODO: Tune parameters for altitude hold mode (barometer filter settings, PID, ...)

// print debug outputs through serial
//#define DEBUG

// plot through Processing
//#define PLOT

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

#ifdef PLOT
	Plotter p;
#endif

// address for eeprom data
#define ADDRESS_EEPROM 128

// LED pin
#define LED_PIN 23

// imu pins
#define IMU_SPI_PORT SPI
#define IMU_CS_PIN 10
#define IMU_INTERRUPT_PIN 9

// barometer pin
#define BAROMETER_INTERRUPT_PIN 20

// pwm pins to control motors
// 1: top-left (CW); 2: top-right (CCW); 3: bottom-left (CW); 4: bottom-right (CCW);
#define MOTOR_PIN_1 2
#define MOTOR_PIN_2 3
#define MOTOR_PIN_3 4
#define MOTOR_PIN_4 5

// motor pwm resolution
#define MOTOR_PWM_RESOLUTION 11

// motor pwm frequency
#define MOTOR_PWM_FREQENCY 4000

// rc channel assignment
#define ROLL		0
#define PITCH		1
#define YAW			3
#define THROTTLE	2
#define ARM			6	// disarm: 1000, enable arming: 2000
#define FMODE		8

#define BETA_INIT	10		// Madgwick algorithm gain (2 * proportional gain (Kp)) during initial pose estimation - 10
#define BETA		0.033	// Madgwick algorithm gain (2 * proportional gain (Kp)) - 0.041 MARG, 0.033 IMU

// time constant for altitude filter
// TC < 1: trust the acceleration measurement more
// TC > 1: trust the position measurement more
#define TC_ALTITUDE_FILTER		0.9	// 0.9

// parameters to check if filtered angles converged during initialisation
#define INIT_ANGLE_DIFFERENCE 0.5		// maximum angle difference between filtered angles and accelerometer angles (x- and y-axis) after initialisation
#define INIT_RATE 0.05	// maximum angular rate of Madgwick-filtered angle (z-axis) after initialisation
#define INIT_VELOCITY_V 0.20	// maximum vertical velocity after initialisation

// flight setpoint limits
#define YAW_RATE_LIMIT		120		// deg/s

#define ROLL_ANGLE_LIMIT	30		// deg
#define PITCH_ANGLE_LIMIT	30		// deg

#define VELOCITY_V_LIMIT	2.5		// m/s

// throttle when armed (slightly above esc/motor deadzone)
#define THROTTLE_ARMED	1125
// Throttle to enter started state and begin PID calculations.
// The throttle stick position is centered around this value.
// To ensure a smooth start, this value should be close to the throttle necessary for take off.
#define THROTTLE_HOVER	1475
// Set throttle limit (< 2000), so there is some headroom for pid control in order to keep the quadcopter stable during full throttle.
#define THROTTLE_LIMIT	1800

// throttle deadzone (altitude hold) in per cent of throttle range
#define THROTTLE_DEADZONE_PCT 20

// minimum time in microseconds a failsafe condition needs to be met, in order to activate it (except for FS_IMU)
#define FS_TIME 500000

// failsafe imu update time limit
#define FS_IMU_DT_LIMIT 2000000

// failsafe motion limits
#define FS_MOTION_ANGLE_LIMIT 60
#define FS_MOTION_RATE_LIMIT 450

// failsafe control limits
#define FS_CONTROL_ANGLE_DIFF 30
#define FS_CONTROL_RATE_DIFF 120

// throttle deadzone limits within which altitude hold is active
const uint16_t THROTTLE_DEADZONE_BOT = 1000 + 10 * (50 - 0.5 * THROTTLE_DEADZONE_PCT);
const uint16_t THROTTLE_DEADZONE_TOP = 1000 + 10 * (50 + 0.5 * THROTTLE_DEADZONE_PCT);

// angle controller acceleration limits (deg/ss)
//const float ACCEL_MIN_ROLL_PITCH = 40;
//const float ACCEL_MIN_YAW = 10;
const float ACCEL_MAX_ROLL_PITCH = 1100;	// 1100, 720
const float ACCEL_MAX_YAW = 180;	// 270, 180
// angle time constant
const float TC_ANGLE = 0.15;

// throttle expo parameter used to achieve less throttle sensitivity around hover
const float THROTTLE_EXPO = 0.3;	// 0.3

// vertical acceleration limit (m/ss)
const float ACCEL_V_MAX = 1;
// altitude time constant
const float TC_ALTITUDE = 2;

// angular rate PID values
const float P_ROLL_RATE = 2.000,	I_ROLL_RATE = 0.000,	D_ROLL_RATE = 0.020;	// 2.500, 0.000, 0.023 @ 0.006 EMA_RATE
const float P_PITCH_RATE = 2.000,	I_PITCH_RATE = 0.000,	D_PITCH_RATE = 0.020;	// 2.500, 0.000, 0.023 @ 0.006 EMA_RATE
const float P_YAW_RATE = 3.500,		I_YAW_RATE = 2.000,		D_YAW_RATE = 0.000;		// 3.500, 2.000, 0.000

// vertical velocity PID values for altitude hold
const float P_VELOCITY_V = 250.000,	I_VELOCITY_V = 5.000,	D_VELOCITY_V = 0.000; 	// 250.000, 5.000, 0.000

// Cut of frequency f_c: https://dsp.stackexchange.com/questions/40462/exponential-moving-average-cut-off-frequency)
// EMA filter parameters for proportional (P) and derivative (D) rate controller inputs.
// EMA = 0.1301 --> f_c = 200 Hz; EMA = 0.0674 --> f_c = 100 Hz;
const float EMA_ROLL_RATE_P		= 0.040;	// 0.040
const float EMA_PITCH_RATE_P	= 0.040;	// 0.040
const float EMA_YAW_RATE_P		= 0.020;	// 0.020
// EMA = 0.0139 --> f_c = 20 Hz; EMA = 0.0035 --> f_c = 5 Hz;
const float EMA_ROLL_RATE_D		= 0.005;	// 0.005
const float EMA_PITCH_RATE_D	= 0.005;	// 0.005
const float EMA_YAW_RATE_D		= 0.005;	// 0.005

// EMA filter parameters for proportional (P)  and derivative (D) vertical velocity controller inputs.
// EMA = 0.1301 --> f_c = 200 Hz; EMA = 0.0674 --> f_c = 100 Hz;
const float EMA_velocity_v_P	= 0.015;	// 0.015
// EMA = 0.0139 --> f_c = 20 Hz; EMA = 0.0035 --> f_c = 5 Hz;
const float EMA_velocity_v_D	= 0.005;	// 0.005

// failsafe configuration
const uint8_t FS_IMU		= 0b00000001;
const uint8_t FS_MOTION		= 0b00000010;
const uint8_t FS_CONTROL	= 0b00000100;

// configure failsafe
const uint8_t FS_CONFIG		= 0b00000011;	// imu and motion failsafe

// list of error codes
const uint8_t ERROR_IMU = 0b00000001;
const uint8_t ERROR_MAG = 0b00000010;
const uint8_t ERROR_BAR = 0b00000100;
// Stores the errors which occurred and disables arming.
// A restart is required to reset the error and enable arming.
uint8_t error_code;

// factors for converting between radians and degrees
const float RAD2DEG = (float) 4068 / 71;
const float DEG2RAD = (float) 71 / 4068;

// configuration data structure
typedef struct {
	// gyroscope offsets in full scale format
	int16_t offset_gx_1000dps, offset_gy_1000dps, offset_gz_1000dps;
	// accelerometer offsets in full scale format
	int16_t offset_ax_32g, offset_ay_32g, offset_az_32g;
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

// enum class for motor states
enum class State { armed, disarmed, arming, disarming };

// motor class
class MotorsQuadcopter {
	public:
	
	// constructor
	MotorsQuadcopter(uint8_t motor1_pin, uint8_t motor2_pin, uint8_t motor3_pin, uint8_t motor4_pin, uint8_t motor_pwm_resolution, uint16_t motor_pwm_frequency) {
		// initialise member variables
		m_motor1_pin = motor1_pin;
		m_motor2_pin = motor2_pin;
		m_motor3_pin = motor3_pin;
		m_motor4_pin = motor4_pin;

		m_motor_pwm_resolution = motor_pwm_resolution;
		m_motor_pwm_frequency = motor_pwm_frequency;

		m_state = State::disarmed;

		// initialise motor pins
		init_pin(m_motor1_pin);
		init_pin(m_motor2_pin);
		init_pin(m_motor3_pin);
		init_pin(m_motor4_pin);
	}

	void output(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4) {
		noInterrupts();
		m_oldResolution = analogWriteResolution(m_motor_pwm_resolution);

		switch (m_state) {
			case State::armed:
				analogWriteMotors(pwm1, pwm2, pwm3, pwm4);
				break;

			case State::disarmed:
				analogWriteMotors(0, 0, 0, 0);
				break;

			case State::arming:
				if (micros() - t0_arming > ARMING_DISARMING_TIME) {
					m_state = State::armed;
					DEBUG_PRINTLN("Armed!");
				}
				else {
					analogWriteMotors(1000, 1000, 1000, 1000);
				}
				break;

			case State::disarming:
				if (micros() - t0_disarming > ARMING_DISARMING_TIME) {
					m_state = State::disarmed;
					DEBUG_PRINTLN("Disarmed!");
				}
				else {
					analogWriteMotors(1000, 1000, 1000, 1000);
				}
				break;
		}

		analogWriteResolution(m_oldResolution);
		interrupts();
	}

	// arm motors
	void arm() {
		// arming is only possible when disarmed and no error has occurred
		if ((m_state == State::disarmed) && (error_code == 0)) {
			t0_arming = micros();
			m_state = State::arming;
		}
	}

	// disarm motors
	void disarm() {
		// disarming when armed or arming (disarm will cancel arming process)
		if ((m_state == State::armed) || (m_state == State::arming)) {
			t0_disarming = micros();
			m_state = State::disarming;
		}
	}

	// return motor state
	State getState() {
		return m_state;
	}
	
	private:

	// initialise motor output pin
	void init_pin(uint8_t pin) {
		analogWriteFrequency(pin, m_motor_pwm_frequency);
		pinMode(pin, OUTPUT);
		digitalWrite(pin, LOW);
	}

	// analogWrite to all motors
	void analogWriteMotors(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4) {
		#if !defined(DEBUG) && !defined(SEND_SERIAL)	// control motors only if debug modes are turned off
			analogWrite(m_motor1_pin, pwm1);
			analogWrite(m_motor2_pin, pwm2);
			analogWrite(m_motor3_pin, pwm3);
			analogWrite(m_motor4_pin, pwm4);
		#endif
	}
	
	// arming and disarming time in microseconds
	const uint32_t ARMING_DISARMING_TIME = 2000000;

	uint8_t m_motor1_pin, m_motor2_pin, m_motor3_pin, m_motor4_pin;
	uint8_t m_motor_pwm_resolution;
	uint16_t m_motor_pwm_frequency;

	// motor state
	State m_state = State::disarmed;

	uint32_t t0_arming, t0_disarming;
	
	uint8_t m_oldResolution;
};

// motor objects
MotorsQuadcopter motors(MOTOR_PIN_1, MOTOR_PIN_2, MOTOR_PIN_3, MOTOR_PIN_4, MOTOR_PWM_RESOLUTION, MOTOR_PWM_FREQENCY);

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

// z-axis pose offset to compensate for the sensor mounting orientation relative to the quadcopter frame
float yaw_angle_offset = 90;

// filtered gyro rates
float roll_rate, pitch_rate, yaw_rate;

// vertical velocity
float velocity_v;

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

// update LED
// mode: 0		off
// mode: 1		blink
// mode: > 1	on
void updateLED(uint8_t pin, uint8_t mode, uint32_t interval_ms = 1000);

// estimate initial pose
void estimatePose(float beta_init, float beta, float init_angleDifference, float init_rate);

// calculate accelerometer x and y angles in degrees
void accelAngles(float& roll_angle_accel, float& pitch_angle_accel);

// multiply two quaternions (Hamilton product)
void qMultiply(float* q1, float* q2, float* result_q);

// calculate acceleration in ned-frame relative to gravity using acceleration in sensor-frame and pose
void calc_accel_ned_rel(float &a_n_rel, float &a_e_rel, float &a_d_rel);

// estimate initial altitude
void estimateAltitude(float init_velocity_v);

// calibrate gyroscope, accelerometer or magnetometer on rc command and return true if any calibration was performed
bool imuCalibration();

// disarm and reset quadcopter
void disarmAndResetQuad();

// arm/disarm on rc command and disarm on failsafe conditions
void arm_failsafe(uint8_t fs_config);

// map input from [in_min, in_max] to [out_min, out_max] and in_between to out_between
float map3(float in, float in_min, float in_between, float in_max, float out_min, float out_between, float out_max);

// generate exponential (cubic) curve
// x = [-1, 1]; expo = [0, 1];
float expo_curve(float x, float expo);

// Calculate the velocity correction from the position error. The velocity has acceleration and deceleration limits including a basic jerk limit using timeConstant.
float shape_position(float position_error, float timeConstant, float accel_max, float last_velocity_sp);

// proportional controller with sqrt sections to constrain the acceleration
float sqrtController(float position_error, float p, float accel_limit);

// limit the acceleration/deceleration of a velocity request
float shape_velocity(float desired_velocity_sp, float accel_max, float last_velocity_sp);

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
	
	// get configuration data from EEPROM
	EEPROM.get(ADDRESS_EEPROM, data_eeprom);
	
	// initialise imu
	if (!imu.init(data_eeprom.offset_gx_1000dps, data_eeprom.offset_gy_1000dps, data_eeprom.offset_gz_1000dps, data_eeprom.offset_ax_32g, data_eeprom.offset_ay_32g, data_eeprom.offset_az_32g, data_eeprom.offset_mx, data_eeprom.offset_my, data_eeprom.offset_mz, data_eeprom.scale_mx, data_eeprom.scale_my, data_eeprom.scale_mz)) {
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
	else if (motors.getState() == State::armed) {
		// blink LED normally to indicate armed status
		updateLED(LED_PIN, 1, 1000);
	}
	else if (motors.getState() == State::disarmed) {
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
	
	// if disarmed, check for calibration request from rc and executed it
	if (imuCalibration()) {
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
	
	roll_rate = gx_rps * RAD2DEG;
	pitch_rate = gy_rps * RAD2DEG;
	yaw_rate = gz_rps * RAD2DEG;
	
	// when armed, calculate flight setpoints, manipulated variables and control motors
	if (motors.getState() == State::armed) {
		
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
							velocity_v_sp = shape_velocity(map(throttle, -1, 1, -VELOCITY_V_LIMIT, VELOCITY_V_LIMIT), ACCEL_V_MAX, velocity_v_sp);
							
							altitude_sp = altitude;
						}
						else {
							// shape vertical velocity to hold altitude
							velocity_v_sp = constrain(shape_position(altitude_sp - altitude, TC_ALTITUDE, ACCEL_V_MAX, velocity_v_sp), -VELOCITY_V_LIMIT, VELOCITY_V_LIMIT);
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
			roll_rate_sp = shape_position(roll_angle_sp - roll_angle, TC_ANGLE, ACCEL_MAX_ROLL_PITCH, roll_rate_sp);
			pitch_rate_sp = shape_position(pitch_angle_sp - pitch_angle, TC_ANGLE, ACCEL_MAX_ROLL_PITCH, pitch_rate_sp);
			
			if (rc_channelValue[THROTTLE] < 1100) {
				// if throttle is too low, disable setting a yaw rate, since it might cause problems when disarming
				yaw_rate_sp = shape_velocity(0, ACCEL_MAX_YAW, yaw_rate_sp);
			} else {
				yaw_rate_sp = shape_velocity(map((float) rc_channelValue[YAW], 1000, 2000, YAW_RATE_LIMIT, -YAW_RATE_LIMIT), ACCEL_MAX_YAW, yaw_rate_sp);
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
		
		// TODO: Check if there is a benefit from magnetometer data
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

// multiply two quaternions (Hamilton product)
void qMultiply(float* q1, float* q2, float* result_q) {
	result_q[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
	result_q[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
	result_q[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
	result_q[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
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
				
				// altitude is estimated if vertical velocity coverged
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

// calibrate gyroscope, accelerometer or magnetometer on rc command and return true if any calibration was performed
bool imuCalibration() {
	static uint32_t t_imuCalibration;
	t_imuCalibration = micros();
	
	static uint32_t t_calibrateGyro = 0, t_calibrateAccel = 0, t_calibrateMag = 0;
	if (motors.getState() == State::disarmed) {
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
					imu.calibrate_gyro(imuInterrupt, 5.0, 1, data_eeprom.offset_gx_1000dps, data_eeprom.offset_gy_1000dps, data_eeprom.offset_gz_1000dps);
					EEPROM.put(ADDRESS_EEPROM, data_eeprom);
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
					imu.calibrate_accel(imuInterrupt, 5.0, 16, data_eeprom.offset_ax_32g, data_eeprom.offset_ay_32g, data_eeprom.offset_az_32g);
					EEPROM.put(ADDRESS_EEPROM, data_eeprom);
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
			if ((rc_channelValue[YAW] > 1900)  && (motors.getState() == State::disarmed)) {
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
			else if ((rc_channelValue[YAW] < 1100) && ((motors.getState() == State::armed) || (motors.getState() == State::arming))) {
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
	else if ((motors.getState() == State::armed) || (motors.getState() == State::arming)) {
		disarmAndResetQuad();
		t_arm = 0;
		t_disarm = 0;
	}
	else {
		t_arm = 0;
		t_disarm = 0;
	}
	
	// -------------------- disarm on failsafe conditions
	if ((motors.getState() == State::armed) || (motors.getState() == State::arming)) {
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

// map input from [in_min, in_max] to [out_min, out_max] and in_between to out_between
float map3(float in, float in_min, float in_between, float in_max, float out_min, float out_between, float out_max) {
	if (in < in_between) {
		return (map(in, in_min, in_between, out_min, out_between));
	}
	else {
		return (map(in, in_between, in_max, out_between, out_max));
	}
}

// Generate exponential (cubic) curve.
// x = [-1, 1]; expo = [0, 1];
float expo_curve(float x, float expo) {
	return (1.0f - expo) * x + expo * x * x * x;
}

// Calculate the velocity correction from the position error. The velocity has acceleration and deceleration limits including a basic jerk limit using timeConstant.
float shape_position(float position_error, float timeConstant, float accel_max, float last_velocity_sp) {
	static float desired_velocity_sp;
	
	// calculate the velocity as position_error approaches zero with acceleration limited by accel_max
	desired_velocity_sp = sqrtController(position_error, 1.0 / max(timeConstant, 0.01), accel_max);
	
	// acceleration is limited directly to smooth the beginning of the curve
	return shape_velocity(desired_velocity_sp, accel_max, last_velocity_sp);
}

// proportional controller with sqrt sections to constrain the acceleration
float sqrtController(float position_error, float p, float accel_max) {
	static float correction_rate;
	static float linear_dist;
	
	linear_dist = accel_max / sq(p);
	
	if (position_error > linear_dist) {
		correction_rate = sqrt(2 * accel_max * (position_error - (linear_dist / 2)));
	}
	else if (position_error < -linear_dist) {
		correction_rate = -sqrt(2 * accel_max * (-position_error - (linear_dist / 2)));
	}
	else {
		correction_rate = position_error * p;
	}
	
	if (dt_s > 0.000010) {
		// this ensures we do not get small oscillations by over shooting the error correction in the last time step
		return constrain(correction_rate, -abs(position_error) / dt_s, abs(position_error) / dt_s);
	}
	else {
		return correction_rate;
	}
}

// limit the acceleration/deceleration of a velocity request
float shape_velocity(float desired_velocity_sp, float accel_max, float last_velocity_sp) {
	static float delta_velocity;
	
	delta_velocity = accel_max * dt_s;
	return constrain(desired_velocity_sp, last_velocity_sp - delta_velocity, last_velocity_sp + delta_velocity);
}
