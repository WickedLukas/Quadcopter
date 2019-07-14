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

// NOTE! Enabling DEBUG adds about 3.3kB to the flash program size.
// Debug output is now working even on ATMega328P MCUs (e.g. Arduino Uno)
// after moving string constants to flash memory storage using the F()
// compiler macro (Arduino IDE 1.0+ required).
//#define DEBUG

// define if you want to send imu data through serial (for example to visualize it in "Processing")
#define SEND_SERIAL

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

// object for ICM-20948 imu
ICM20948_SPI imu(IMU_CS_PIN, IMU_SPI_PORT);

// object for Madgwick filter
MADGWICK_AHRS madgwickFilter(0.041);

// initialization flag, necessary to calculate starting values
boolean first = true;

// measured imu update time in microseconds
int32_t dt = 0;
// measured imu update time in seconds
float dt_s = 0;

// imu measurements
int16_t ax, ay, az;
float gx_rps, gy_rps, gz_rps;
int16_t mx, my, mz;
// last imu measurements
int16_t ax0, ay0, az0;
float gx0_rps, gy0_rps, gz0_rps;
int16_t mx0, my0, mz0;

// imu interrupt flag
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
    static int8_t imuStatus;
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
}

void loop() {
    // variables to measure imu update time
    static uint32_t t0 = 0;
    static uint32_t t = 0;
    
    // flag for new magnetometer data
    static bool new_mag;
    
    // quadcopter pose
    static float angle_x, angle_y, angle_z;
    
    while (first) {        
        while (!imuInterrupt) {
            // wait for next imu interrupt
        }
        // reset imu interrupt flag
        imuInterrupt = false;
        
        t0 = micros();
        
        // read imu measurements
        imu.read_accel_gyro_rps(ax0, ay0, az0, gx0_rps, gy0_rps, gz0_rps);
        new_mag = imu.read_mag(mx0, my0, mz0);
        
        // continue if new magnetometer data is available
        if (new_mag) {
            first = false;
        }
    }
    
    while (!imuInterrupt) {
        // wait for next imu interrupt
    }
    
    // reset imu interrupt flag
    imuInterrupt = false;
    
    t = micros();
    
    // read imu measurements
    imu.read_accel_gyro_rps(ax, ay, az, gx_rps, gy_rps, gz_rps);
    new_mag = imu.read_mag(mx, my, mz);
    
    dt = (t - t0);                  // in us
    dt_s = (float) (dt) * 1.e-6;    // in s
    t0 = t;                         // update last imu update time measurement
    
    madgwickFilter.get_euler(angle_x, angle_y, angle_z, dt_s, ax, ay, az, gx_rps, gy_rps, gz_rps, mx, -my, -mz);
    
    
    
    gx0_rps = gx_rps;
    gy0_rps = gy_rps;
    gz0_rps = gz_rps;
    
    first = false;


    if (first | ((ax != ax0) | (ay != ay0) | (az != az0))) {
        //DEBUG_PRINT("a "); DEBUG_PRINT(t_a);
        //DEBUG_PRINT("\t"); DEBUG_PRINT(ax); DEBUG_PRINT("\t"); DEBUG_PRINT(ay); DEBUG_PRINT("\t"); DEBUG_PRINTLN(az);
        
        ax0 = ax;
        ay0 = ay;
        az0 = az;
    }
    
    if (new_mag) {
        //DEBUG_PRINT("m "); DEBUG_PRINT(t_m);
        //DEBUG_PRINT("\t"); DEBUG_PRINT(mx); DEBUG_PRINT("\t"); DEBUG_PRINT(my); DEBUG_PRINT("\t"); DEBUG_PRINTLN(mz);
                
        mx0 = mx;
        my0 = my;
        mz0 = mz;
    }
   
   
   // run serial print at a rate independent of the main loop
   static uint32_t t0_serial = micros();
   if (micros() - t0_serial > 16666) {
        t0_serial = micros();
        
        //DEBUG_PRINT(angle_x); DEBUG_PRINT("\t"); DEBUG_PRINT(angle_y); DEBUG_PRINT("\t"); DEBUG_PRINTLN(angle_z);
        
        //DEBUG_PRINTLN();
        
        //DEBUG_PRINT(ax); DEBUG_PRINT("\t"); DEBUG_PRINT(ay); DEBUG_PRINT("\t"); DEBUG_PRINTLN(az);
        //DEBUG_PRINT(gx_rps); DEBUG_PRINT("\t"); DEBUG_PRINT(gy_rps); DEBUG_PRINT("\t"); DEBUG_PRINTLN(gz_rps);
        //DEBUG_PRINT(mx); DEBUG_PRINT("\t"); DEBUG_PRINT(my); DEBUG_PRINT("\t"); DEBUG_PRINTLN(mz);
        
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
