/*
* Quadcopter.ino
*
* Created: 26.04.2019
* Author: Lukas
* 
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

// pins connected to IMU
#define IMU_SPI_PORT SPI
#define IMU_CS_PIN 10
#define IMU_INTERRUPT_PIN 24

// Object for ICM-20948 sensor on I2C bus 1
// IMU I2C address has to be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
ICM20948_SPI IMU(IMU_CS_PIN, IMU_SPI_PORT);

// measured IMU update time in microseconds
int32_t dt = 0;
// measured IMU update time in seconds
float dT = 0;

// IMU raw measurements
int16_t ax, ay, az;     // accelerometer
int16_t gx, gy, gz;     // gyroscope
int16_t mx, my, mz;     // magnetometer
int16_t temperature;    // temperature

volatile bool imuInterrupt = false;     // indicates whether IMU interrupt pin has gone high
void imuReady() {
    imuInterrupt = true;
}

void setup() {
    #ifdef DEBUG
        // initialize serial communication
        Serial.begin(115200);
        while (!Serial);
    #endif

    static int8_t imuStatus;
    // start communication with IMU
    imuStatus = IMU.init();
    if (!imuStatus) {
        DEBUG_PRINTLN("IMU initialization unsuccessful");
        DEBUG_PRINTLN("Check IMU wiring or try cycling power");
        while(1) {}
    }
  
    // Setup interrupt pin
    pinMode(IMU_INTERRUPT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), imuReady, FALLING);
}

void loop() {
    static uint32_t t0 = 0;
    static uint32_t t = 0;

    while (!imuInterrupt) {
        // wait for the next interrupt
    }
    imuInterrupt = false;
    
    IMU.read_sensors(ax, ay, az, gx, gy, gz, temperature, mx, my, mz);
    
    t = micros();
    dt = (t - t0);              // in us
    dT = float(dt) * 0.000001;  // in s
  
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
