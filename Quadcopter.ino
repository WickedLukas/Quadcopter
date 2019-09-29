/*
* Quadcopter.ino
*
* Created:	26.04.2019
* Author:	Lukas
*
*/
#include <Arduino.h>
#include <EEPROM.h>
// TODO: Maybe it can be avoided to use this library
#include <PWMServo.h>

#include "ICM20948.h"
#include "MadgwickAHRS.h"
#include "iBus.h"
#include "ema_filter.h"
#include "PID_controller.h"

#include "sendSerial.h"

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

// address for data saved to eeprom
#define ADDRESS_EEPROM 0

// pins connected to imu
#define IMU_SPI_PORT SPI
#define IMU_CS_PIN 10
#define IMU_INTERRUPT_PIN 24

#define BETA_INIT 10	// Madgwick algorithm gain (2 * proportional gain (Kp)) during initial pose estimation
#define BETA 0.041		// Madgwick algorithm gain (2 * proportional gain (Kp))

// parameters to check if filtered angles converged during initialization
#define INIT_ANGLE_DIFFERENCE 0.5	// Maximum angle difference between filtered angles and accelerometer angles (x- and y-axis) after initialization
#define INIT_ANGULAR_VELOCITY 0.05	// Maximum angular velocity of filtered angle (z-axis) after initialization

// rc channel assignment
#define ROLL		0
#define PITCH		1
#define YAW			3
#define THROTTLE	2

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
float P_roll = 1,	I_roll = 0,		D_roll = 0;
float P_pitch = 1,	I_pitch = 0,	D_pitch = 0;
float P_yaw = 1,	I_yaw = 0,		D_yaw = 0;

// object for ICM-20948 imu
ICM20948_SPI imu(IMU_CS_PIN, IMU_SPI_PORT);

// object for Madgwick filter
MADGWICK_AHRS madgwickFilter(BETA_INIT);

// object for radio control (rc)
IBUS rc;

// PID controller for pose
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

// imu interrupt
volatile bool imuInterrupt = false;
void imuReady() {
	imuInterrupt = true;
}

// calculate accelerometer x and y angles in degrees
void accelAngles(float& angle_x_accel, float& angle_y_accel);

// calculate flight setpoints from rc input
void flight_setpoints(float& roll_sp, float& pitch_sp, float& yaw_sp, float& throttle_sp);

void setup() {
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
	Serial3.begin(115200, SERIAL_8N1);
	while (!Serial3);
	
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
	
	// initialize rc and return a pointer on the received rc channel values
	rc_channelValue = rc.begin(Serial3);
	
	// setup interrupt pin
	pinMode(IMU_INTERRUPT_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), imuReady, FALLING);
	
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
	
	dt = (t - t0);					// in us
	dt_s = (float) (dt) * 1.e-6;	// in s
	t0 = t;							// update last imu update time measurement
	
	madgwickFilter.get_euler(dt_s, ax, ay, az, gx_rps, gy_rps, gz_rps, mx, my, mz, angle_x, angle_y, angle_z);
	
	// update rc
	rc.update();
	
	// calculate flight setpoints
	static float roll_sp, pitch_sp, yaw_sp, throttle_sp;
	flight_setpoints(roll_sp, pitch_sp, yaw_sp, throttle_sp);
	
	// get manipulated variables
	static float roll_mv, pitch_mv, yaw_mv;
	roll_mv = roll_pid.get_mv(roll_sp, angle_x, dt_s);
	pitch_mv = pitch_pid.get_mv(pitch_sp, angle_y, dt_s);
	yaw_mv = yaw_pid.get_mv(yaw_sp, angle_z, dt_s);
	
	// TODO: Throttle necessary hold altitude depends on roll and pitch angle, so it might be useful to compensate for this
	
	
	
	
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
		
		//DEBUG_PRINTLN();
		
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


// calculate accelerometer x and y angles in degrees
void accelAngles(float& angle_x_accel, float& angle_y_accel) {
	static const float RAD2DEG = 4068 / 71;
	
	angle_x_accel = atan2(ay, az) * RAD2DEG;
	angle_y_accel = atan2(-ax, sqrt(pow(ay,2) + pow(az,2))) * RAD2DEG;
}

// calculate flight setpoints from rc input
void flight_setpoints(float& roll_sp, float& pitch_sp, float& yaw_sp, float& throttle_sp) {
	static float roll_mapped, pitch_mapped, yaw_mapped, throttle_mapped;
	
	// map rc channel values
	roll_mapped = map((float) rc_channelValue[ROLL], 1000, 2000, -ROLL_LIMIT, ROLL_LIMIT);
	pitch_mapped = map((float) rc_channelValue[PITCH], 1000, 2000, -PITCH_LIMIT, PITCH_LIMIT);
	yaw_mapped = map((float) rc_channelValue[YAW], 1000, 2000, -YAW_LIMIT, YAW_LIMIT);
	throttle_mapped = map((float) rc_channelValue[THROTTLE], 1000, 2000, 1000, THROTTLE_LIMIT);
	
	// calculate flight setpoints by filtering the mapped rc channel values
	roll_sp = ema_filter(roll_mapped, roll_sp, EMA_ROLL_SP);
	pitch_sp = ema_filter(pitch_mapped, pitch_sp, EMA_PITCH_SP);
	yaw_sp = ema_filter(yaw_mapped, roll_sp, EMA_YAW_SP);
	throttle_sp = ema_filter(throttle_mapped, throttle_sp, EMA_THROTTLE_SP);
}