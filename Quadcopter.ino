/*
* Quadcopter.ino
*
* Created: 26.04.2019
* Author: Lukas
* 
*/
#include <Arduino.h>

#include "ICM20948.h"
#include "MadgwickAHRS.h"

#include "sendSerial.h"

// calculate accelerometer x and y angles in degrees
void calc_accelAngles(float& angle_x_accel, float& angle_y_accel);

// NOTE! Enabling DEBUG adds about 3.3kB to the flash program size.
// Debug output is now working even on ATMega328P MCUs (e.g. Arduino Uno)
// after moving string constants to flash memory storage using the F()
// compiler macro (Arduino IDE 1.0+ required).
#define DEBUG

// define if you want to send imu data through serial (for example to visualize it in "Processing")
//#define SEND_SERIAL

// define if you want to calibrate the imu
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

// pins connected to imu
#define IMU_SPI_PORT SPI
#define IMU_CS_PIN 10
#define IMU_INTERRUPT_PIN 24

#define BETA_INIT 50	// Madgwick algorithm gain (2 * proportional gain (Kp)) during initial pose estimation
#define BETA 0.041		// Madgwick algorithm gain (2 * proportional gain (Kp))

#define INIT_ANGLE_DIFF -1  // Maximum angle difference between the accelerometer angle and the filtered angle after initialization

// object for ICM-20948 imu
ICM20948_SPI imu(IMU_CS_PIN, IMU_SPI_PORT);

// object for Madgwick filter
MADGWICK_AHRS madgwickFilter(BETA_INIT);

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

void setup() {
	#ifdef DEBUG
		// initialize serial communication
		Serial.begin(115200);
		while (!Serial);
	#endif
	
	IMU_SPI_PORT.begin();

	// initialize imu
	int8_t imuStatus;
	imuStatus = imu.init();
	
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
		imu.calibrate_mag(imuInterrupt, 60, 0);
	#endif
	
	// initial pose estimation flag
	bool init = true;
	// angles calculated from accelerometer
	float angle_x_accel, angle_y_accel;
	
	DEBUG_PRINTLN(F("Estimating initial pose. Keep device at rest ..."));
	
	// estimate initial pose
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
		
		madgwickFilter.get_euler(angle_x, angle_y, angle_z, dt_s, ax, ay, az, gx_rps, gy_rps, gz_rps, mx, my, mz);
		
		//madgwickFilter.set_angle_z(0);
		
		calc_accelAngles(angle_x_accel, angle_y_accel);
		
		if ((abs(angle_x_accel - angle_x) < INIT_ANGLE_DIFF) && (abs(angle_y_accel - angle_y) < INIT_ANGLE_DIFF)) {
			madgwickFilter.set_beta(BETA);
			
			init = false;
			
			DEBUG_PRINTLN(F("Initial pose estimated."));
			DEBUG_PRINTLN(abs(angle_x_accel - angle_x));
			DEBUG_PRINTLN(abs(angle_y_accel - angle_y));
		}
		
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
	
	madgwickFilter.get_euler(angle_x, angle_y, angle_z, dt_s, ax, ay, az, gx_rps, gy_rps, gz_rps, mx, my, mz);
	
	
	
	// run serial print at a rate independent of the main loop
	static uint32_t t0_serial = micros();
	if (micros() - t0_serial > 16666) {
		t0_serial = micros();
		
		//DEBUG_PRINT(ax); DEBUG_PRINT("\t"); DEBUG_PRINT(ay); DEBUG_PRINT("\t"); DEBUG_PRINTLN(az);
		//DEBUG_PRINT(gx_rps); DEBUG_PRINT("\t"); DEBUG_PRINT(gy_rps); DEBUG_PRINT("\t"); DEBUG_PRINTLN(gz_rps);
		//DEBUG_PRINT(mx); DEBUG_PRINT("\t"); DEBUG_PRINT(my); DEBUG_PRINT("\t"); DEBUG_PRINTLN(mz);
		
		static float angle_x_accel, angle_y_accel;
		calc_accelAngles(angle_x_accel, angle_y_accel);
		DEBUG_PRINT(angle_x_accel); DEBUG_PRINT("\t"); DEBUG_PRINT(angle_y_accel); DEBUG_PRINT("\t"); DEBUG_PRINTLN(0);
		DEBUG_PRINT(angle_x); DEBUG_PRINT("\t"); DEBUG_PRINT(angle_y); DEBUG_PRINT("\t"); DEBUG_PRINTLN(angle_z);
		
		//DEBUG_PRINT(ax); DEBUG_PRINT("\t"); DEBUG_PRINT(ay); DEBUG_PRINT("\t"); DEBUG_PRINTLN(az);
		//DEBUG_PRINT(gx_rps); DEBUG_PRINT("\t"); DEBUG_PRINT(gy_rps); DEBUG_PRINT("\t"); DEBUG_PRINTLN(gz_rps);
		//DEBUG_PRINT(mx); DEBUG_PRINT("\t"); DEBUG_PRINT(my); DEBUG_PRINT("\t"); DEBUG_PRINTLN(mz);
		
		DEBUG_PRINTLN();
		
		//DEBUG_PRINT(ax_g); DEBUG_PRINT("\t"); DEBUG_PRINT(ay_g); DEBUG_PRINT("\t"); DEBUG_PRINTLN(az_g);
		//DEBUG_PRINT(mx_ut); DEBUG_PRINT("\t"); DEBUG_PRINT(my_ut); DEBUG_PRINT("\t"); DEBUG_PRINTLN(mz_ut);
		
		/*DEBUG_PRINT(dt);
		DEBUG_PRINT("\t");
		DEBUG_PRINT2(imu.getAccelX_mss(),2);
		DEBUG_PRINT("\t");
		DEBUG_PRINT2(imu.getAccelY_mss(),2);
		DEBUG_PRINT("\t");
		DEBUG_PRINT2(imu.getAccelZ_mss(),2);
		DEBUG_PRINT("\t");
		DEBUG_PRINT2(imu.getGyroX_rads(),2);
		DEBUG_PRINT("\t");
		DEBUG_PRINT2(imu.getGyroY_rads(),2);
		DEBUG_PRINT("\t");
		DEBUG_PRINT2(imu.getGyroZ_rads(),2);
		DEBUG_PRINT("\t");
		DEBUG_PRINT2(imu.getMagX_uT(),2);
		DEBUG_PRINT("\t");
		DEBUG_PRINT2(imu.getMagY_uT(),2);
		DEBUG_PRINT("\t");
		DEBUG_PRINT2(imu.getMagZ_uT(),2);
		DEBUG_PRINT("\t");
		DEBUG_PRINTLN2(imu.getTemperature_C(),2);*/
		
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
