#include <Arduino.h>
#include <EEPROM.h>

#include "iBus.h"
#include "ICM20948.h"
#include "BMP388_DEV.h"
#include "MadgwickAHRS.h"
#include "ema_filter.h"
#include "PID_controller.h"

#include "Plotter.h"
#include "sendSerial.h"

// TODO: integrate telemetry (MAVLink?)

// print debug outputs through serial
//#define DEBUG

// plot through Processing
#define PLOT

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
#define MOTOR_PWM_FREQENCY 12000

// rc channel assignment
#define ROLL		0
#define PITCH		1
#define YAW			3
#define THROTTLE	2
#define ARM			6	// disarmed: 1000, armed: 2000
#define FMODE		8	// stable: 1000/2000, stable with tilt compensated thrust: 1500, acro (not implemented)

#define BETA_INIT	10		// Madgwick algorithm gain (2 * proportional gain (Kp)) during initial pose estimation - 10
#define BETA		0.041	// Madgwick algorithm gain (2 * proportional gain (Kp)) - 0.041 MARG, 0.033 IMU

// parameters to check if filtered angles converged during initialisation
#define INIT_ANGLE_DIFFERENCE 0.5		// maximum angle difference between filtered angles and accelerometer angles (x- and y-axis) after initialisation
#define INIT_RATE 0.05	// maximum angular rate of Madgwick-filtered angle (z-axis) after initialisation

// limits for the flight setpoints
#define YAW_RATE_LIMIT		180		// deg/s

#define ROLL_ANGLE_LIMIT	30		// deg
#define PITCH_ANGLE_LIMIT	30		// deg

// Throttle to enter started state and begin PID calculations.
// The throttle stick position is centered around this value.
// To ensure a smooth start, this value should be close to the throttle necessary for take off.
#define THROTTLE_HOVER	1400
// Set throttle limit (< 2000), so there is some headroom for pid control in order to keep the quadcopter stable during full throttle.
#define THROTTLE_LIMIT	1750

// angle controller acceleration limits (deg/ss)
//const float ACCEL_MIN_ROLL_PITCH = 40;
const float ACCEL_MAX_ROLL_PITCH = 1100;	// 720
//const float ACCEL_MIN_YAW = 10;
const float ACCEL_MAX_YAW = 270;	// 120

// angle controller time constant
const float TIME_CONSTANT = 0.15;

// angular rate PID values
const float P_ROLL_RATE = 2.500,	I_ROLL_RATE = 0.000,	D_ROLL_RATE = 0.023; 	// 2.500, 0.000, 0.023 @ 0.006 EMA_RATE 
const float P_PITCH_RATE = 2.500,	I_PITCH_RATE = 0.000,	D_PITCH_RATE = 0.023;
const float P_YAW_RATE = 1.000,		I_YAW_RATE = 0.000,		D_YAW_RATE = 0.000;		// 0.200, 0.000, 0.000

// moving average filter configuration for the angular rates (gyro)
// TODO: Use notch filter or a band stop filter from two EMA filters with specified cut off frequencies.
const float EMA_ROLL_RATE	= 0.006;
const float EMA_PITCH_RATE	= 0.006;
const float EMA_YAW_RATE	= 0.006;

// failsafe configuration
const uint8_t FS_IMU		= 0b00000001;
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

// factors for converting bettween radians and degrees
const float RAD2DEG = (float) 4068 / 71;
const float DEG2RAD = (float) 71 / 4068;

// list of error codes
const uint8_t ERROR_IMU = 0b00000001;
const uint8_t ERROR_MAG = 0b00000010;
const uint8_t ERROR_BAR = 0b00000100;
// Stores the errors which occurred and disables arming.
// A restart is required to reset the error and enable arming.
uint8_t error_code = 0;

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

// rate PID controller
PID_controller roll_rate_pid(P_ROLL_RATE, I_ROLL_RATE, D_ROLL_RATE, 0, 0, 2000);
PID_controller pitch_rate_pid(P_PITCH_RATE, I_PITCH_RATE, D_PITCH_RATE, 0, 0, 2000);
PID_controller yaw_rate_pid(P_YAW_RATE, I_YAW_RATE, D_YAW_RATE, 0, 0, 2000);

// variables to measure imu update time
uint32_t t0 = 0, t = 0;
// measured imu update time in microseconds
int32_t dt = 0;
// measured imu update time in seconds
float dt_s = 0;

// pointer on an array of 10 received rc channel values [1000; 2000]
uint16_t *rc_channelValue;

// flight setpoints
float throttle_sp;
float roll_angle_sp, pitch_angle_sp;
float roll_rate_sp, pitch_rate_sp, yaw_rate_sp;

// manipulated variables
float roll_rate_mv, pitch_rate_mv, yaw_rate_mv;

// accelerometer resolution
float accelRes;

// imu measurements
int16_t ax, ay, az;
float gx_rps, gy_rps, gz_rps;
int16_t mx = 0, my = 0, mz = 0;

// barometer altitude measurement
float baroAltitude;

// quadcopter pose
float roll_angle, pitch_angle, yaw_angle;	// euler angles
float pose_q[4];							// quaternion

// quadcopter altitude
float altitude1, altitude2, altitude3;

// TODO: Remove this later
float a_ned_rel_q0, a_ned_rel_q1, a_ned_rel_q2, a_ned_rel_q3;

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

// calculate altitude in m from acceleration, barometer altitude and pose
void calcAltitude();

// calibrate gyroscope, accelerometer or magnetometer on rc command and return true if any calibration was performed
bool imuCalibration();

// disarm and reset quadcopter
void disarmAndResetQuad();

// arm/disarm on rc command and disarm on failsafe conditions
void arm_failsafe(uint8_t fs_config);

// Calculate the rate correction from the angle error. The rate has acceleration and deceleration limits including a basic jerk limit using timeConstant.
float shape_angle(float error_angle, float timeConstant, float accel_max, float last_rate_sp);

// proportional controller with sqrt sections to constrain the angular acceleration
float sqrtController(float error_angle, float p, float accel_limit);

// limit the acceleration/deceleration of a rate request
float shape_rate(float desired_rate_sp, float accel_max, float last_rate_sp);

void setup() {
	// setup built in LED
	pinMode(LED_PIN, OUTPUT);
	
	// turn off LED
	updateLED(LED_PIN, 0);
	
	// set default resolution for analog write, in order to go back to it after running motors with different resolution
	analogWriteResolution(8);
	
	#if defined(DEBUG) || defined(SEND_SERIAL) || defined(PLOT)
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
	if (!imu.init(data_eeprom.offset_gx_1000dps, data_eeprom.offset_gy_1000dps, data_eeprom.offset_gz_1000dps, data_eeprom.offset_ax_32g, data_eeprom.offset_ay_32g, data_eeprom.offset_az_32g, data_eeprom.offset_mx, data_eeprom.offset_my, data_eeprom.offset_mz, data_eeprom.scale_mx, data_eeprom.scale_my, data_eeprom.scale_mz)) {
		// IMU could not be initialized. Set error value, which will disable arming.
		error_code = error_code | ERROR_IMU;
		DEBUG_PRINTLN("IMU error: Initialisation failed!");
	}
	
	// read accelerometer resolution in g/bit
	imu.read_accelRes(accelRes);
	
	// initialise BMP388 mode, pressure oversampling, temperature oversampling, IIR-Filter and standby time
	if (!bmp388.begin(NORMAL_MODE, OVERSAMPLING_X8, OVERSAMPLING_X1, IIR_FILTER_2, TIME_STANDBY_20MS)) {
		// barometer could not be initialized. Set error value, which will disable arming.
		error_code = error_code | ERROR_BAR;
		DEBUG_PRINTLN("BAR error: Initialisation failed!");
	}
	
	// setup interrupt pin for BMP388
	pinMode(BAROMETER_INTERRUPT_PIN, INPUT);
	bmp388.enableInterrupt();
	attachInterrupt(digitalPinToInterrupt(BAROMETER_INTERRUPT_PIN), barometerReady, RISING);
	
	#ifdef PLOT
		// !Start plotter
		p.Begin();
		
		// Add time graphs. Notice the effect of points displayed on the time scale
		//p.AddTimeGraph("Angles", 1000, "roll_angle", roll_angle, "pitch_angle", pitch_angle, "yaw_angle", yaw_angle);
		//p.AddTimeGraph("Barometer altitude", 1000, "baroAltitude", baroAltitude);
		p.AddTimeGraph("Quadcopter altitude", 10000, "altitude1", altitude1, "altitude2", altitude2, "altitude3", altitude3);
		//p.AddTimeGraph("Relative acceleration in ned-frame", 10000, "a_ned_rel_q1", a_ned_rel_q1, "a_ned_rel_q2", a_ned_rel_q2, "a_ned_rel_q3", a_ned_rel_q3);
	#endif
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
	madgwickFilter.get_euler_quaternion(dt_s, ax, ay, az, gx_rps, gy_rps, gz_rps, mx, my, mz, roll_angle, pitch_angle, yaw_angle, pose_q);
	
	// apply offset to z-axis pose in order to compensate for the sensor mounting orientation relative to the quadcopter frame
	yaw_angle += yaw_angle_offset;
	if (yaw_angle > 180) {
		yaw_angle -= 360;
	}
	else if (yaw_angle < -180) {
		yaw_angle += 360;
	}
	
	// calculate altitude in m from acceleration, barometer altitude and pose
	calcAltitude();
	
	// filter gyro rates
	// TODO: Use a notch filter or a bandstop filter made from a combination of two EMA filters instead
	roll_rate = ema_filter(gx_rps * RAD2DEG, roll_rate, EMA_ROLL_RATE);
	pitch_rate = ema_filter(gy_rps * RAD2DEG, pitch_rate, EMA_PITCH_RATE);
	yaw_rate = ema_filter(gz_rps * RAD2DEG, yaw_rate, EMA_YAW_RATE);
	
	// when armed, calculate flight setpoints, manipulated variables and control motors
	if (armed) {
		// throttle setpoint
		if (rc_channelValue[THROTTLE] < 1500) {
			throttle_sp = map((float) rc_channelValue[THROTTLE], 1000, 1500, 1000, THROTTLE_HOVER);
		}
		else {
			throttle_sp = map((float) rc_channelValue[THROTTLE], 1500, 2000, THROTTLE_HOVER, THROTTLE_LIMIT);
		}
		
		if (rc_channelValue[FMODE] == 1500) {
			// Stable flightmode with tilt compensated thrust: Increase thrust when quadcopter is tilted, to compensate for height loss during horizontal movement.
			// Note: In order to maintain stability, tilt compensated thrust is limited to the throttle limit.
			throttle_sp = constrain((float) (throttle_sp - 1000) / (pose_q[0]*pose_q[0] - pose_q[1]*pose_q[1] - pose_q[2]*pose_q[2] + pose_q[3]*pose_q[3]) + 1000, 1000, THROTTLE_LIMIT);
		}
		
		// angle setpoints
		roll_angle_sp = map((float) rc_channelValue[ROLL], 1000, 2000, -ROLL_ANGLE_LIMIT, ROLL_ANGLE_LIMIT);
		pitch_angle_sp = map((float) rc_channelValue[PITCH], 1000, 2000, -PITCH_ANGLE_LIMIT, PITCH_ANGLE_LIMIT);
		
		// rate setpoints
		roll_rate_sp = shape_angle(roll_angle_sp - roll_angle, TIME_CONSTANT, ACCEL_MAX_ROLL_PITCH, roll_rate_sp);
		pitch_rate_sp = shape_angle(pitch_angle_sp - pitch_angle, TIME_CONSTANT, ACCEL_MAX_ROLL_PITCH, pitch_rate_sp);
		yaw_rate_sp = shape_rate(map((float) rc_channelValue[YAW], 1000, 2000, -YAW_RATE_LIMIT, YAW_RATE_LIMIT), ACCEL_MAX_YAW, yaw_rate_sp);
		
		
		// TODO: Remove this test code
		//static float p_rate, i_rate, d_rate;
		//p_rate = map((float) rc_channelValue[4], 1000, 2000, 2.5, 5);
		//i_rate = map((float) rc_channelValue[5], 1000, 2000, 0, 1);
		//d_rate = map((float) rc_channelValue[5], 1000, 2000, 0.023, 0.05);
		
		//roll_rate_pid.set_K_p(p_rate);
		//roll_rate_pid.set_K_i(i_rate);
		//roll_rate_pid.set_K_d(d_rate);
		
		//pitch_rate_pid.set_K_p(p_rate);
		//pitch_rate_pid.set_K_i(i_rate);
		//pitch_rate_pid.set_K_d(d_rate);
		
		// In order to ensure a smooth start, PID calculations are delayed until a minimum throttle value is applied.
		if (started) {
			// calculate manipulated variables
			roll_rate_mv = roll_rate_pid.get_mv(roll_rate_sp, roll_rate, dt_s);
			pitch_rate_mv = pitch_rate_pid.get_mv(pitch_rate_sp, pitch_rate, dt_s);
			yaw_rate_mv = yaw_rate_pid.get_mv(yaw_rate_sp, yaw_rate, dt_s);
			
			motor_1.write(constrain(throttle_sp + roll_rate_mv - pitch_rate_mv + yaw_rate_mv, 1000, 2000));
			motor_2.write(constrain(throttle_sp - roll_rate_mv - pitch_rate_mv - yaw_rate_mv, 1000, 2000));
			motor_3.write(constrain(throttle_sp - roll_rate_mv + pitch_rate_mv + yaw_rate_mv, 1000, 2000));
			motor_4.write(constrain(throttle_sp + roll_rate_mv + pitch_rate_mv - yaw_rate_mv, 1000, 2000));
		}
		else if (throttle_sp > THROTTLE_HOVER) {
			started = true;
			DEBUG_PRINTLN("Started!");
		}
		else {
			motor_1.write(1000);
			motor_2.write(1000);
			motor_3.write(1000);
			motor_4.write(1000);
		}
	}
	else {
		// for safety reasons repeat disarm and reset, even when it was already done
		disarmAndResetQuad();
	}
	
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
			//DEBUG_PRINT(map((float) rc_channelValue[THROTTLE], 1000, 2000, 1000, THROTTLE_LIMIT)); DEBUG_PRINT("\t"); DEBUG_PRINTLN(throttle_sp);
			//DEBUG_PRINT(map((float) (rc_channelValue[THROTTLE] - 1000) / (cos(roll_angle * DEG2RAD) * cos(pitch_angle * DEG2RAD)) + 1000, 1000, 2000, 1000, THROTTLE_LIMIT)); DEBUG_PRINT("\t"); DEBUG_PRINTLN(throttle_sp);
			
			//DEBUG_PRINTLN(roll_rate_sp);
			
			//DEBUG_PRINT(roll_angle); DEBUG_PRINT("\t"); DEBUG_PRINT(pitch_angle); DEBUG_PRINT("\t"); DEBUG_PRINT(yaw_angle); DEBUG_PRINT("\t");
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
		if (micros() - t0_serial > 1000) {
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
		
		madgwickFilter.get_euler_quaternion(dt_s, ax, ay, az, gx_rps, gy_rps, gz_rps, mx, my, mz, roll_angle, pitch_angle, yaw_angle, pose_q);
		
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

// multiply two quaternions (Hamilton product)
void qMultiply(float* q1, float* q2, float* result_q) {
	result_q[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
	result_q[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
	result_q[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
	result_q[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
}

// calculate altitude in m from acceleration, barometer altitude and pose
void calcAltitude() {
	// acceleration of gravity is m/s²
	const double G = 9.81;
	
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
	
	// Calculate relative acceleration by removing gravity and transforming acceleration from bit to m/s²
	a_ned_rel_q0 = a_ned_q[0] * accelRes * G;
	a_ned_rel_q1 = a_ned_q[1] * accelRes * G;
	a_ned_rel_q2 = a_ned_q[2] * accelRes * G;
	a_ned_rel_q3 = (a_ned_q[3] * accelRes - 1) * G;
	
	// A Comparison of Complementary and Kalman Filtering
	// WALTER T. HIGGINS, JR.
	
	// k1 = sqrt(2 * std_w / std_v)
	// k2 = std_w / std_v
	// std_w: standard deviation in the noise of the acceleration (0.1 m/s²)
	// std_v: standard deviation in the noise of the barometer altitude (0.11 m)
	
	static float ratio1 = 0.45, ratio2 = 0.9, ratio3 = 1.8;	// 0.9091
	
	static const float k1_1 = sqrt(2 * ratio1);	// 1.3484	
	static const float k2_1 = ratio1;			// 0.9091
	
	static const float k1_2 = sqrt(2 * ratio2);	
	static const float k2_2 = ratio2;
	
	static const float k1_3 = sqrt(2 * ratio3);	
	static const float k2_3 = ratio3;
	
	static float velocity1, velocity2, velocity3;	// vertical velocity
	altitude1 += dt_s * velocity1 + (k1_1 + 0.5 * dt_s * k2_1) * dt_s * (baroAltitude - altitude1) + 0.5 * dt_s * a_ned_rel_q3 * dt_s;
	velocity1 += k2_1 * dt_s * (baroAltitude - altitude1) + a_ned_rel_q3 * dt_s;
	
	altitude2 += dt_s * velocity2 + (k1_2 + 0.5 * dt_s * k2_2) * dt_s * (baroAltitude - altitude2) + 0.5 * dt_s * a_ned_rel_q3 * dt_s;
	velocity2 += k2_2 * dt_s * (baroAltitude - altitude2) + a_ned_rel_q3 * dt_s;
	
	altitude3 += dt_s * velocity3 + (k1_3 + 0.5 * dt_s * k2_3) * dt_s * (baroAltitude - altitude3) + 0.5 * dt_s * a_ned_rel_q3 * dt_s;
	velocity3 += k2_3 * dt_s * (baroAltitude - altitude3) + a_ned_rel_q3 * dt_s;
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
		
		throttle_sp = 1000;
		
		// reset PID controller
		roll_rate_pid.reset();
		pitch_rate_pid.reset();
		yaw_rate_pid.reset();
		
		roll_rate_mv = 0;
		pitch_rate_mv = 0;
		yaw_rate_mv = 0;
		
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

// Calculate the rate correction from the angle error. The rate has acceleration and deceleration limits including a basic jerk limit using timeConstant.
float shape_angle(float error_angle, float timeConstant, float accel_max, float last_rate_sp) {
	static float desired_rate_sp;
	
	// calculate the rate as error_angle approaches zero with acceleration limited by accel_max (Ardupilot uses rad/ss but we use deg/ss here)
	desired_rate_sp = sqrtController(error_angle, 1.0 / max(timeConstant, 0.01), accel_max);
	
	// acceleration is limited directly to smooth the beginning of the curve
	return shape_rate(desired_rate_sp, accel_max, last_rate_sp);
}

// proportional controller with sqrt sections to constrain the angular acceleration
float sqrtController(float error_angle, float p, float accel_max) {
	static float correction_rate;
	static float linear_dist;
	
	linear_dist = accel_max / sq(p);
	
	if (error_angle > linear_dist) {
		correction_rate = sqrt(2 * accel_max * (error_angle - (linear_dist / 2)));
	}
	else if (error_angle < -linear_dist) {
		correction_rate = -sqrt(2 * accel_max * (-error_angle - (linear_dist / 2)));
	}
	else {
		correction_rate = error_angle * p;
	}
	
	if (dt_s > 0.000010) {
		// this ensures we do not get small oscillations by over shooting the error correction in the last time step
		return constrain(correction_rate, -abs(error_angle) / dt_s, abs(error_angle) / dt_s);
	}
	else {
		return correction_rate;
	}
}

// limit the acceleration/deceleration of a rate request
float shape_rate(float desired_rate_sp, float accel_max, float last_rate_sp) {
	static float delta_rate;
	
	delta_rate	= accel_max * dt_s;
	return constrain(desired_rate_sp, last_rate_sp - delta_rate, last_rate_sp + delta_rate);
}
