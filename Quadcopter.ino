/*
* Quadcopter.ino
*
* Created:	26.04.2019
* Author:	Lukas
*
*/
#include <Arduino.h>
#include <EEPROM.h>

#include "ICM20948.h"
#include "MadgwickAHRS.h"
#include "iBus.h"
#include "ema_filter.h"
#include "PID_controller.h"

#include "sendSerial.h"

// TODO: integrate telemetry (MAVLink?)
// TODO: arming sequence
// TODO: LED notification

// print debug outputs through serial
#define DEBUG

// send imu data through serial (for example to visualize it in "Processing")
//#define SEND_SERIAL

// calibrate imu
//#define IMU_CALIBRATION

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

// parameters to check if filtered angles converged during initialization
#define INIT_ANGLE_DIFFERENCE 0.5	// Maximum angle difference between filtered angles and accelerometer angles (x- and y-axis) after initialization
#define INIT_ANGULAR_VELOCITY 0.05	// Maximum angular velocity of filtered angle (z-axis) after initialization

// limits for flight setpoints
#define ROLL_LIMIT		33		// deg
#define PITCH_LIMIT		33		// deg
#define YAW_LIMIT		180		// deg/s
#define THROTTLE_LIMIT	1750	// < 2000 because there is some headroom needed for pid control, so the quadcopter stays stable during full throttle

// moving average filter configuration for the flight setpoints
#define EMA_ROLL_SP			0.0002
#define EMA_PITCH_SP		0.0002
#define EMA_YAW_SP			0.0002
#define EMA_THROTTLE_SP		0.0002

//TODO: Check if it is better to use a cascaded PID loop controlling rotational rate and angle
// PID values for pose controller
const float P_roll = 1,		I_roll = 0,		D_roll = 0;
const float P_pitch = 1,	I_pitch = 0,	D_pitch = 0;
const float P_yaw = 1,		I_yaw = 0,		D_yaw = 0;
//const float P_level = 1,	I_level = 0,	D_level = 0;

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
	static uint8_t m_oldRes;
	uint8_t m_pin;
	uint8_t m_motor_pwm_resolution;
};

uint8_t PWMServoMotor::m_oldRes;

// ICM-20948 imu object
ICM20948_SPI imu(IMU_CS_PIN, IMU_SPI_PORT);

// motor objects
PWMServoMotor motor_1(MOTOR_PIN_1, MOTOR_PWM_RESOLUTION, MOTOR_PWM_FREQENCY);
PWMServoMotor motor_2(MOTOR_PIN_2, MOTOR_PWM_RESOLUTION, MOTOR_PWM_FREQENCY);
PWMServoMotor motor_3(MOTOR_PIN_3, MOTOR_PWM_RESOLUTION, MOTOR_PWM_FREQENCY);
PWMServoMotor motor_4(MOTOR_PIN_4, MOTOR_PWM_RESOLUTION, MOTOR_PWM_FREQENCY);

// Madgwick filter object
MADGWICK_AHRS madgwickFilter(BETA_INIT);

// radio control (rc) object
IBUS rc;

// pose PID controller
PID_controller roll_pid(P_roll, I_roll, D_roll, 0, 0, 2000);
PID_controller pitch_pid(P_pitch, I_pitch, D_yaw, 0, 0, 2000);
PID_controller yaw_pid(P_yaw, I_yaw, D_yaw, 0, 0, 2000);

// configuration data structure
typedef struct {
	// magnetometer hard iron distortion correction
	float offset_mx, offset_my, offset_mz;
	// magnetometer soft iron distortion correction
	float scale_mx, scale_my, scale_mz;
} config_data;

// motors can only run when armed
bool armed = false;

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

// pointer on an array of 10 received rc channel values [1000; 2000]
uint16_t *rc_channelValue;

// quadcopter pose
float angle_x, angle_y, angle_z;

// z-axis pose offset to compensate for the sensor mounting orientation relative to the quadcopter frame
float angle_z_offset = 90;

// imu interrupt
volatile bool imuInterrupt = false;
void imuReady() {
	imuInterrupt = true;
}

// arm/disarm on rc command and disarm on failsafe conditions
void arm_failsafe();

// calculate accelerometer x and y angles in degrees
void accelAngles(float& angle_x_accel, float& angle_y_accel);

// calculate flight setpoints from rc input for stable flightmode without and with tilt compensated thrust
void flightSetpoints(float& roll_sp, float& pitch_sp, float& yaw_sp, float& throttle_sp);

void setup() {
	// set default resolution for analog write, in order to go back to it after running motors with different resolution
	analogWriteResolution(8);
	
	// configuration data
	config_data data_eeprom;
	
	// last quadcopter z-axis angle
	float angle_z_0 = 0;
	
	#if defined(DEBUG) || defined(SEND_SERIAL) || defined(IMU_CALIBRATION)
		// initialize serial for monitoring
		Serial.begin(115200);
		while (!Serial);
	#endif
	
	// initialize serial for iBus communication
	Serial2.begin(115200, SERIAL_8N1);
	while (!Serial2);
	
	// initialize SPI for imu communication
	IMU_SPI_PORT.begin();
	
	// get configuration data from EEPROM
	EEPROM.get(ADDRESS_EEPROM, data_eeprom);
	
	// initialize imu
	int8_t imuStatus;
	imuStatus = imu.init(data_eeprom.offset_mx, data_eeprom.offset_my, data_eeprom.offset_mz, data_eeprom.scale_mx, data_eeprom.scale_my, data_eeprom.scale_mz);
	
	if (!imuStatus) {
		DEBUG_PRINTLN(F("IMU initialization failed."));
		while(1) {}
	}
	
	// setup interrupt pin
	pinMode(IMU_INTERRUPT_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), imuReady, FALLING);
	
	// initialize rc and return a pointer on the received rc channel values
	rc_channelValue = rc.begin(Serial2);
	
	// TODO: Implement a proper startup procedure here, which also gives the option for calibration and guides the user with display, sounds or LEDs
	
	// calibrate imu
	#ifdef IMU_CALIBRATION
		//imu.reset_accel_gyro_offsets();
		//imu.calibrate_gyro(imuInterrupt, 5.0, 1);
		//imu.calibrate_accel_gyro(imuInterrupt, 5.0, 16, 1);
		imu.calibrate_mag(imuInterrupt, 60, 0, data_eeprom.offset_mx, data_eeprom.offset_my, data_eeprom.offset_mz, data_eeprom.scale_mx, data_eeprom.scale_my, data_eeprom.scale_mz);
		
		// save configuration data to eeprom
		EEPROM.put(ADDRESS_EEPROM, data_eeprom);
	#endif
	
	// initial pose estimation flag
	bool init = true;
	// angles calculated from accelerometer
	float angle_x_accel, angle_y_accel;
	
	// estimate initial pose
	DEBUG_PRINTLN(F("Estimating initial pose. Keep device at rest ..."));
	while (init) {
		while (!imuInterrupt) {
			// wait for next imu interrupt
		}
		// reset imu interrupt flag
		imuInterrupt = false;
		
		t = micros();
		
		// read imu measurements
		imu.read_accel_gyro_rps(ax, ay, az, gx_rps, gy_rps, gz_rps);
		imu.read_mag(mx, my, mz);
		
		dt = (t - t0);					// in us
		dt_s = (float) (dt) * 1.e-6;	// in s
		t0 = t;							// update last imu update time measurement
		
		madgwickFilter.get_euler(dt_s, ax, ay, az, gx_rps, gy_rps, gz_rps, mx, my, mz, angle_x, angle_y, angle_z);
		
		accelAngles(angle_x_accel, angle_y_accel);
		
		// initialization is completed if filtered angles converged
		if ((abs(angle_x_accel - angle_x) < INIT_ANGLE_DIFFERENCE) && (abs(angle_y_accel - angle_y) < INIT_ANGLE_DIFFERENCE)
		&& ((abs(angle_z - angle_z_0) / dt_s) < INIT_ANGULAR_VELOCITY)) {
			// reduce beta value, since filtered angles have stabilized during initialization
			madgwickFilter.set_beta(BETA);
			
			init = false;
			
			DEBUG_PRINTLN(F("Initial pose estimated."));
			DEBUG_PRINTLN2(abs(angle_x_accel - angle_x), 6);
			DEBUG_PRINTLN2(abs(angle_y_accel - angle_y), 6);
			DEBUG_PRINTLN2(abs(angle_z - angle_z_0) / dt_s, 6);
		}
		angle_z_0 = angle_z;
		
		
		// run serial print at a rate independent of the main loop
		static uint32_t t0_serial = micros();
		if (micros() - t0_serial > 16666) {
			t0_serial = micros();
			
			DEBUG_PRINT(angle_x_accel); DEBUG_PRINT("\t"); DEBUG_PRINTLN(angle_y_accel);
			DEBUG_PRINT(angle_x); DEBUG_PRINT("\t"); DEBUG_PRINT(angle_y); DEBUG_PRINT("\t"); DEBUG_PRINTLN(angle_z);
		}
	}
}

void loop() {
	while (!imuInterrupt) {
		// wait for next imu interrupt
	}
	
	// reset imu interrupt flag
	imuInterrupt = false;
	
	t = micros();
	
	// read imu measurements
	imu.read_accel_gyro_rps(ax, ay, az, gx_rps, gy_rps, gz_rps);
	imu.read_mag(mx, my, mz);
	
	dt = (t - t0);  // in us
	dt_s = (float) (dt) * 1.e-6;	// in s
	t0 = t; // update last imu update time measurement
	
	madgwickFilter.get_euler(dt_s, ax, ay, az, gx_rps, gy_rps, gz_rps, mx, my, mz, angle_x, angle_y, angle_z);
	
	// apply offset to z-axis pose in order to compensate for the sensor mounting orientation relative to the quadcopter frame
	angle_z += angle_z_offset;
	if (angle_z > 180) {
		angle_z -= 360;
	}
	else if (angle_z < -180) {
		angle_z += 360;
	}
	
	// update rc
	rc.update();
	
	// arm/disarm on rc command and disarm on failsafe conditions
	arm_failsafe();
	
	// when armed, calculate flight setpoints, manipulated variables and control motors
	static float roll_sp, pitch_sp, yaw_sp, throttle_sp;	// flight setpoints
	static float roll_mv, pitch_mv, yaw_mv;	// manipulated variables
	if (armed) {
		// calculate flight setpoints
		flightSetpoints(roll_sp, pitch_sp, yaw_sp, throttle_sp);
		
		// get manipulated variables
		roll_mv = roll_pid.get_mv(roll_sp, angle_x, dt_s);
		pitch_mv = pitch_pid.get_mv(pitch_sp, angle_y, dt_s);
		yaw_mv = yaw_pid.get_mv(yaw_sp, angle_z, dt_s);
		
		// TODO: Throttle necessary hold altitude depends on roll and pitch angle, so it might be useful to compensate for this
		// TODO: Check motor mixing logic carefully
		/*motor_1 = throttle_sp + roll_mv - pitch_mv + yaw_mv;
		motor_2 = throttle_sp - roll_mv - pitch_mv - yaw_mv;
		motor_3 = throttle_sp + roll_mv + pitch_mv + yaw_mv;
		motor_4 = throttle_sp - roll_mv + pitch_mv - yaw_mv;*/
		
		motor_1.write(rc_channelValue[THROTTLE]);
		/*motor_2.write(rc_channelValue[THROTTLE]);
		motor_3.write(rc_channelValue[THROTTLE]);
		motor_4.write(rc_channelValue[THROTTLE]);*/
	}
	else {
		// set flight setpoints to zero
		roll_sp = 0;
		pitch_sp = 0;
		yaw_sp = 0;
		throttle_sp = 0;
		
		flightSetpoints(roll_sp, pitch_sp, yaw_sp, throttle_sp);
		
		// reset PIDs
		roll_pid.reset();
		pitch_pid.reset();
		yaw_pid.reset();
		
		// turn off motors
		motor_1.write(0);
		motor_2.write(0);
		motor_3.write(0);
		motor_4.write(0);
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
		DEBUG_PRINTLN(armed);
		
		DEBUG_PRINTLN();
		
		// print channel values
		for (int i=0; i<10 ; i++) {
			DEBUG_PRINT(rc_channelValue[i]); DEBUG_PRINT("\t");
		}
		DEBUG_PRINTLN();
		
		#ifdef SEND_SERIAL
			// Send data to "Processing" for visualization
			sendSerial(dt, angle_x, angle_y, angle_z);
		#endif
	}
}


// arm/disarm on rc command and disarm on failsafe conditions
void arm_failsafe() {
	// arm and disarm on rc command
	static uint32_t t_arm;
	static uint32_t t_disarm;

	if (rc_channelValue[ARM] == 2000) {
		// arm:			left stick bottom-right	(2s)
		// disarm:	left stick bottom-left	(2s)
		if (rc_channelValue[THROTTLE] < 1050) {
			if (rc_channelValue[YAW] > 1950) {
			  // hold stick to complete disarming
				t_arm += dt;
				t_disarm = 0;
				
				if (t_arm > 2000000) {
						armed = true;
				}
			} 
			else if (rc_channelValue[YAW] < 1050) {
				// hold stick to complete arming
				t_disarm += dt;
				t_arm = 0;
				
				if (t_disarm > 2000000) {
					armed = false;
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
		armed = false;
	}

	// TODO: disarm on failsafe conditions
	
}

// calculate accelerometer x and y angles in degrees
void accelAngles(float& angle_x_accel, float& angle_y_accel) {
	static const float RAD2DEG = 4068 / 71;
	
	angle_x_accel = atan2(ay, az) * RAD2DEG;
	angle_y_accel = atan2(-ax, sqrt(pow(ay,2) + pow(az,2))) * RAD2DEG;
}

// calculate flight setpoints from rc input for stable flightmode without and with tilt compensated thrust
void flightSetpoints(float& roll_sp, float& pitch_sp, float& yaw_sp, float& throttle_sp) {
	static const float DEG2RAD = 71 / 4068;
	static float roll_mapped, pitch_mapped, yaw_mapped, throttle_mapped;
	
	// map rc channel values
	roll_mapped = map((float) rc_channelValue[ROLL], 1000, 2000, -ROLL_LIMIT, ROLL_LIMIT);
	pitch_mapped = map((float) rc_channelValue[PITCH], 1000, 2000, -PITCH_LIMIT, PITCH_LIMIT);
	yaw_mapped = map((float) rc_channelValue[YAW], 1000, 2000, -YAW_LIMIT, YAW_LIMIT);

	if (rc_channelValue[FMODE] == 1500) {
		// tilt compensated thrust: increase thrust when quadcopter is tilted, to compensate for height loss during horizontal movement
		throttle_mapped = map((float) rc_channelValue[THROTTLE] / (cos(angle_x * DEG2RAD) * cos(angle_y * DEG2RAD)), 1000, 2000, 1000, THROTTLE_LIMIT);
	}
	else {
		// stable
		throttle_mapped = map((float) rc_channelValue[THROTTLE], 1000, 2000, 1000, THROTTLE_LIMIT);
	}
	
	// calculate flight setpoints by filtering the mapped rc channel values
	roll_sp = ema_filter(roll_mapped, roll_sp, EMA_ROLL_SP);
	pitch_sp = ema_filter(pitch_mapped, pitch_sp, EMA_PITCH_SP);
	yaw_sp = ema_filter(yaw_mapped, roll_sp, EMA_YAW_SP);
	throttle_sp = ema_filter(throttle_mapped, throttle_sp, EMA_THROTTLE_SP);
}
