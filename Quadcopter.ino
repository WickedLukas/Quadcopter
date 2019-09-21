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

#include "sendSerial.h"

// print debug outputs through serial
//#define DEBUG

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

// object for ICM-20948 imu
ICM20948_SPI imu(IMU_CS_PIN, IMU_SPI_PORT);

// object for Madgwick filter
MADGWICK_AHRS madgwickFilter(BETA_INIT);

// object for remote control
IBUS remoteControl;

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

// quadcopter pose
float angle_x, angle_y, angle_z;

// imu interrupt
volatile bool imuInterrupt = false;
void imuReady() {
	imuInterrupt = true;
}

// calculate accelerometer x and y angles in degrees
void calc_accelAngles(float& angle_x_accel, float& angle_y_accel);

void setup() {
	// configuration data
	config_data data_eeprom;
	
	// last quadcopter z-axis angle
	float angle_z_0 = 0;
	
	#if defined(DEBUG) || defined(SEND_SERIAL) || defined(IMU_CALIBRATION)
		// initialize serial communication
		Serial.begin(115200);
		while (!Serial);
	#endif
	
	IMU_SPI_PORT.begin();
	
	// get configuration data from eeprom
	EEPROM.get(ADDRESS_EEPROM, data_eeprom);
	
	// initialize imu
	int8_t imuStatus;
	imuStatus = imu.init(data_eeprom.offset_mx, data_eeprom.offset_my, data_eeprom.offset_mz, data_eeprom.scale_mx, data_eeprom.scale_my, data_eeprom.scale_mz);
	
	if (!imuStatus) {
		DEBUG_PRINTLN("imu initialization unsuccessful");
		DEBUG_PRINTLN("Check imu wiring or try cycling power");
		while(1) {}
	}
	
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
		
		calc_accelAngles(angle_x_accel, angle_y_accel);
		
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
			
			DEBUG_PRINT(angle_x); DEBUG_PRINT("\t"); DEBUG_PRINT(angle_y); DEBUG_PRINT("\t"); DEBUG_PRINTLN(angle_z);
			DEBUG_PRINT(angle_x_accel); DEBUG_PRINT("\t"); DEBUG_PRINT(angle_y_accel); DEBUG_PRINT("\t"); DEBUG_PRINTLN(0);
			DEBUG_PRINTLN();
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
	
	
	
	// run serial print at a rate independent of the main loop (t0_serial = 16666 for 60 Hz update rate)
	static uint32_t t0_serial = micros();
	if (micros() - t0_serial > 16666) {
		t0_serial = micros();
		
		//DEBUG_PRINT(ax); DEBUG_PRINT("\t"); DEBUG_PRINT(ay); DEBUG_PRINT("\t"); DEBUG_PRINTLN(az);
		//DEBUG_PRINT(gx_rps); DEBUG_PRINT("\t"); DEBUG_PRINT(gy_rps); DEBUG_PRINT("\t"); DEBUG_PRINTLN(gz_rps);
		//DEBUG_PRINT(mx); DEBUG_PRINT("\t"); DEBUG_PRINT(my); DEBUG_PRINT("\t"); DEBUG_PRINTLN(mz);
		
		static float angle_x_accel, angle_y_accel;
		calc_accelAngles(angle_x_accel, angle_y_accel);
		//DEBUG_PRINT(angle_x_accel); DEBUG_PRINT("\t"); DEBUG_PRINT(angle_y_accel); DEBUG_PRINT("\t"); DEBUG_PRINTLN(0);
		//DEBUG_PRINT(angle_x); DEBUG_PRINT("\t"); DEBUG_PRINT(angle_y); DEBUG_PRINT("\t"); DEBUG_PRINTLN(angle_z);
		
		//DEBUG_PRINTLN();
		
		#ifdef SEND_SERIAL
			// Send data to "Processing" for visualization
			sendSerial(dt, angle_x, angle_y, angle_z);
		#endif
	}
}


// calculate accelerometer x and y angles in degrees
void calc_accelAngles(float& angle_x_accel, float& angle_y_accel) {
	static const float RAD2DEG = 4068 / 71;
	
	angle_x_accel = atan2(ay, az) * RAD2DEG;
	angle_y_accel = atan2(-ax, sqrt(pow(ay,2) + pow(az,2))) * RAD2DEG;
}
