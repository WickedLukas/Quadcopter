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

// define if you want to calibrate the IMU
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

// initialization flag, necessary to calculate starting values
boolean first = true;

// measured imu update time in microseconds
int32_t dt = 0;
// measured imu update time in seconds
float dt_s = 0;

// imu measurements
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
// last imu measurements
int16_t ax0, ay0, az0;
int16_t gx0, gy0, gz0;
int16_t mx0, my0, mz0;

// imu measurements in units
/*
float ax_g, ay_g, az_g;         // accelerometer measurement in g
float gx_dps, gy_dps, gz_dps;   // gyroscope measurement in deg/s
float mx_ut, my_ut, mz_ut;      // magnetometer measurement in uT
float temperature_c;            // temperature measurement in C
*/

volatile bool imuInterrupt = false;     // indicates whether imu interrupt pin has gone high
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

    static int8_t imuStatus;
    // start communication with imu
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
        imu.calibrate_mag(imuInterrupt, 20, 0);
    #endif
}

void loop() {
    // variables to measure imu update time
    static uint32_t t0 = 0;
    static uint32_t t = 0;
    
    // flag to indicate new magnetometer data
    static bool new_mag;
    
    while (first) {        
        while (!imuInterrupt) {
            // wait for next imu interrupt
        }
        // reset imu interrupt flag
        imuInterrupt = false;
        
        t0 = micros();
        
        // read imu measurements
        imu.read_accel_gyro(ax0, ay0, az0, gx0, gy0, gz0);
        new_mag = imu.read_mag(mx0, my0, mz0);
        //imu.read_gyro_dps_accel_g(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g);
        //new_mag = imu.read_mag_ut(mx_ut, my_ut, mz_ut);
        
        // continue if new magnetometer data is available
        if (new_mag) {
            first = false;
        }
    }
    
    static uint32_t t_a = 0;    // time since last accelerometer measurement
    static uint32_t t_m = 0;    // time since last gyroscope measurement
    
    while (!imuInterrupt) {
        // wait for next imu interrupt
    }
    // reset imu interrupt flag
    imuInterrupt = false;
    
    t = micros();
    
    // read imu measurements
    imu.read_accel_gyro(ax, ay, az, gx, gy, gz);
    new_mag = imu.read_mag(mx, my, mz);
    //imu.read_gyro_dps_accel_g(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g);
    //new_mag = imu.read_mag_ut(mx_ut, my_ut, mz_ut);
    
    dt = (t - t0);                  // in us
    dt_s = (float) (dt) * 1.e-6;    // in s
    t0 = t;                         // update last imu update time measurement
    
    t_a += dt;
    t_m += dt;
    
    //DEBUG_PRINT("g "); DEBUG_PRINT(dt);
    //DEBUG_PRINT("\t"); DEBUG_PRINT(gx); DEBUG_PRINT("\t"); DEBUG_PRINT(gy); DEBUG_PRINT("\t"); DEBUG_PRINTLN(gz);
    
    gx0 = gx;
    gy0 = gy;
    gz0 = gz;
    
    if (first | ((ax != ax0) | (ay != ay0) | (az != az0))) {
        //DEBUG_PRINT("a "); DEBUG_PRINT(t_a);
        //DEBUG_PRINT("\t"); DEBUG_PRINT(ax); DEBUG_PRINT("\t"); DEBUG_PRINT(ay); DEBUG_PRINT("\t"); DEBUG_PRINTLN(az);
        
        t_a = 0;
        
        ax0 = ax;
        ay0 = ay;
        az0 = az;
    }
    
    if (new_mag) {
        DEBUG_PRINT("m "); DEBUG_PRINT(t_m);
        DEBUG_PRINT("\t"); DEBUG_PRINT(mx); DEBUG_PRINT("\t"); DEBUG_PRINT(my); DEBUG_PRINT("\t"); DEBUG_PRINTLN(mz);
        
        t_m = 0;
                
        mx0 = mx;
        my0 = my;
        mz0 = mz;
    }
    
    //DEBUG_PRINTLN();
    
    first = false;
    
    //DEBUG_PRINT(ax); DEBUG_PRINT("\t"); DEBUG_PRINT(ay); DEBUG_PRINT("\t"); DEBUG_PRINTLN(az);
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
}