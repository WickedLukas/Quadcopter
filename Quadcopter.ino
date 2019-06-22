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

// object for ICM-20948 IMU
ICM20948_SPI IMU(IMU_CS_PIN, IMU_SPI_PORT);

// initialization flag, necessary to calculate starting values
boolean first = true;

// measured IMU update time in microseconds
int32_t dt = 0;
// measured IMU update time in seconds
float dT = 0;

// IMU measurements
int16_t ax, ay, az;     // accelerometer
int16_t gx, gy, gz;     // gyroscope
int16_t mx, my, mz;     // magnetometer
int16_t temperature;    // temperature

// IMU measurements in units
/*
float ax_g, ay_g, az_g;         // accelerometer measurement in g
float gx_dps, gy_dps, gz_dps;   // gyroscope measurement in deg/s
float mx_ut, my_ut, mz_ut;      // magnetometer measurement in uT
float temperature_c;            // temperature measurement in C
*/

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
    
    IMU_SPI_PORT.begin();

    static int8_t imuStatus;
    // start communication with IMU
    imuStatus = IMU.init();
    
    if (!imuStatus) {
        DEBUG_PRINTLN("IMU initialization unsuccessful");
        DEBUG_PRINTLN("Check IMU wiring or try cycling power");
        while(1) {}
    }
  
    // setup interrupt pin
    pinMode(IMU_INTERRUPT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), imuReady, FALLING);
}

void loop() {
    // variables to measure IMU update time
    static uint32_t t0 = 0;
    static uint32_t t = 0;
    
    // flag to indicate new magnetometer data
    static bool new_mag;
    
    while (first) {        
        while (!imuInterrupt) {
            // wait for next IMU interrupt
        }
        // reset IMU interrupt flag
        imuInterrupt = false;
        
        t0 = micros();
        
        // read IMU measurements
        IMU.read_gyro_accel(gx, gy, gz, ax, ay, az);
        new_mag = IMU.read_mag(mx, my, mz);
        //IMU.read_gyro_dps_accel_g(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g);
        //new_mag = IMU.read_mag_ut(mx_ut, my_ut, mz_ut);
        
        // continue if new magnetometer data is available
        if (new_mag) {
            first = false;
        }
    }
    
    while (!imuInterrupt) {
        // wait for next IMU interrupt
    }
    // reset IMU interrupt flag
    imuInterrupt = false;
    
    t = micros();
    
    // read IMU measurements
    IMU.read_gyro_accel(gx, gy, gz, ax, ay, az);
    new_mag = IMU.read_mag(mx, my, mz);
    //IMU.read_gyro_dps_accel_g(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g);
    //new_mag = IMU.read_mag_ut(mx_ut, my_ut, mz_ut);
    
    dt = (t - t0);              // in us
    dT = (float) (dt) * 1.e-6;  // in s
    t0 = t;                     // update last IMU update time measurement
    
    
    // display the data
    static int16_t ax_old = ax;
    static int16_t ay_old = ay;
    static int16_t az_old = az;
    static int16_t gx_old = gx;
    static int16_t gy_old = gy;
    static int16_t gz_old = gz;
    static int16_t mx_old = mx;
    static int16_t my_old = my;
    static int16_t mz_old = mz;
    static int16_t temperature_old = temperature;
    
    static uint32_t t_g = 0;
    static uint32_t t_a = 0;
    static uint32_t t_m = 0;
    static uint32_t t_temperature = 0;
    
    t_g += dt;
    t_a += dt;
    t_m += dt;
    t_temperature += dt;
    
    if (first | ((gx != gx_old) | (gy != gy_old) | (gz != gz_old))) {
        //DEBUG_PRINT("g "); DEBUG_PRINTLN(t_g);
        //DEBUG_PRINT("\t"); DEBUG_PRINT(gx); DEBUG_PRINT("\t"); DEBUG_PRINT(gy); DEBUG_PRINT("\t"); DEBUG_PRINTLN(gz);
        
        t_g = 0;
    }
    
    if (first | ((ax != ax_old) | (ay != ay_old) | (az != az_old))) {
        //DEBUG_PRINT("a "); DEBUG_PRINTLN(t_a);
        //DEBUG_PRINT("\t"); DEBUG_PRINT(ax); DEBUG_PRINT("\t"); DEBUG_PRINT(ay); DEBUG_PRINT("\t"); DEBUG_PRINTLN(az);
        
        t_a = 0;
    }
    
    if (new_mag) {
        //DEBUG_PRINT("m "); DEBUG_PRINTLN(t_m);
        DEBUG_PRINT(mx); DEBUG_PRINT("\t"); DEBUG_PRINT(my); DEBUG_PRINT("\t"); DEBUG_PRINTLN(mz);
        
        t_m = 0;
    }
    
    if (first | (temperature != temperature_old)) {
        //DEBUG_PRINT("t "); DEBUG_PRINTLN(t_temperature);
        //DEBUG_PRINT("\t"); DEBUG_PRINTLN(temperature);
        
        t_temperature = 0;
    }
    
    //DEBUG_PRINTLN();
    
    ax_old = ax;
    ay_old = ay;
    az_old = az;
    gx_old = gx;
    gy_old = gy;
    gz_old = gz;
    mx_old = mx;
    my_old = my;
    mz_old = mz;
    temperature_old = temperature;
    
    //DEBUG_PRINT(ax); DEBUG_PRINT("\t"); DEBUG_PRINT(ay); DEBUG_PRINT("\t"); DEBUG_PRINTLN(az);
    //DEBUG_PRINT(ax_g); DEBUG_PRINT("\t"); DEBUG_PRINT(ay_g); DEBUG_PRINT("\t"); DEBUG_PRINTLN(az_g);
    //DEBUG_PRINT(mx_ut); DEBUG_PRINT("\t"); DEBUG_PRINT(my_ut); DEBUG_PRINT("\t"); DEBUG_PRINTLN(mz_ut);
    
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
