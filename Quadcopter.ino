/*
* Quadcopter.ino
*
* Created: 26.04.2019
* Author: Lukas
* 
* Using ICM-20948 Arduino library from Sparkfun
*/

#include "ICM20948.h"

// NOTE! Enabling DEBUG adds about 3.3kB to the flash program size.
// Debug output is now working even on ATMega328P MCUs (e.g. Arduino Uno)
// after moving string constants to flash memory storage using the F()
// compiler macro (Arduino IDE 1.0+ required).
#define DEBUG

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

#define IMU_INTERRUPT_PIN 36

// Object for ICM-20948 sensor on I2C bus 1
// IMU I2C address has to be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
ICM20948 IMU(Wire1, 0x68);

// measured IMU update time in microseconds
int32_t dt = 0;
// measured IMU update time in seconds
float dT = 0;

// IMU raw measurements
int16_t ax, ay, az;		// accelerometer
int16_t gx, gy, gz;		// gyroscope
int16_t hx, hy, hz;		// magnetometer

volatile bool imuInterrupt = false;		// indicates whether IMU interrupt pin has gone high
void imuReady() {
  imuInterrupt = true;
}

void setup() {
  #ifdef DEBUG
    // initialize serial communication
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo eNUMeration, others continue immediately
  #endif

  static int8_t imuStatus;
  // start communication with IMU 
  imuStatus = IMU.begin();
  DEBUG_PRINT("imuStatus = ");
  DEBUG_PRINTLN(imuStatus);
  if (imuStatus < 0) {
    DEBUG_PRINTLN("IMU initialization unsuccessful");
    DEBUG_PRINTLN("Check IMU wiring or try cycling power");
    while(1) {}
  }

  // TODO:
  // - odr_align_en to sync sample rates seems not to be necessary
  IMU.configGyro(ICM20948::GYRO_RANGE_1000DPS, ICM20948::GYRO_DLPF_BANDWIDTH_197HZ);
  IMU.configAccel(ICM20948::ACCEL_RANGE_8G, ICM20948::ACCEL_DLPF_BANDWIDTH_246HZ);
  
  // the gyroscope sample rate is 8000 Hz for GYRO_DLPF_BANDWIDTH_12106HZ else 1125 / (1 + Srd) Hz
  //IMU.setGyroSrd(0); // Output data rate is 1125/(1 + srd) Hz
  
  // the accelerometer sample rate is 4500 Hz for ACCEL_DLPF_BANDWIDTH_1209HZ else 1125 / (1 + Srd) Hz
  //IMU.setAccelSrd(0);

  IMU.enableDataReadyInterrupt();
  
  pinMode(IMU_INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), imuReady, RISING);
}

void loop() {
  static uint32_t t0 = 0;
  static uint32_t t = 0;

  while (!imuInterrupt) {
    // wait for the next interrupt
  }
  imuInterrupt = false;
  
  IMU.readSensor();
	
  t = micros();
  dt = (t - t0);				// in us
  dT = float(dt) * 0.000001;	// in s
  
  // update last IMU update time measurement
  t0 = t;

  // display the data
  DEBUG_PRINTLN(dt);
  
  /*DEBUG_PRINT(dt);
  DEBUG_PRINT("\t");
  DEBUG_PRINT2(IMU.getAccelX_mss(),2);
  DEBUG_PRINT("\t");
  DEBUG_PRINT2(IMU.getAccelY_mss(),2);
  DEBUG_PRINT("\t");
  DEBUG_PRINT2(IMU.getAccelZ_mss(),2);
  DEBUG_PRINT("\t");
  DEBUG_PRINT2(IMU.getGyroX_rads(),2);
  DEBUG_PRINT("\t");
  DEBUG_PRINT2(IMU.getGyroY_rads(),2);
  DEBUG_PRINT("\t");
  DEBUG_PRINT2(IMU.getGyroZ_rads(),2);
  DEBUG_PRINT("\t");
  DEBUG_PRINT2(IMU.getMagX_uT(),2);
  DEBUG_PRINT("\t");
  DEBUG_PRINT2(IMU.getMagY_uT(),2);
  DEBUG_PRINT("\t");
  DEBUG_PRINT2(IMU.getMagZ_uT(),2);
  DEBUG_PRINT("\t");
  DEBUG_PRINTLN2(IMU.getTemperature_C(),2);*/
}
