#ifndef MAIN_H_
#define MAIN_H_

#include "common.h"

#include <Plotter.h>
#include <NMEAGPS.h>

#include <stdint.h>

// TODO: Tune parameters for altitude hold mode (barometer filter settings, PID, ...).

#if defined(USE_GPS) && (!defined(USE_MAG) || !defined(USE_BAR))
#error If USE_GPS is defined USE_MAGNETOMETER and USE_BAR must be defined!
#endif

// z-axis pose offset to compensate for the sensor mounting orientation relative to the quadcopter frame
#ifdef USE_MAG
    #define YAW_ANGLE_OFFSET 0
#else
    #define YAW_ANGLE_OFFSET 90
#endif

// imu pins
#define IMU_SPI_PORT SPI
#define IMU_CS_PIN 10
#define IMU_INTERRUPT_PIN 9

// barometer interrupt pin
#define BAROMETER_INTERRUPT_PIN 22

// pwm pins to control motors
// 1: top-left (CW); 2: top-right (CCW); 3: bottom-left (CW); 4: bottom-right (CCW);
#define MOTOR_PIN_1 2
#define MOTOR_PIN_2 3
#define MOTOR_PIN_3 4
#define MOTOR_PIN_4 5

// motor pwm frequency
#define MOTOR_PWM_FREQUENCY 4000

// Madgwick algorithm gain beta (2 * proportional gain (Kp))
#define BETA_INIT 10 // beta during initial pose estimation
#ifdef USE_MAG
    #define BETA 0.033 // MARG 0.041
#else
    #define BETA 0.033 // IMU 0.033
#endif

// parameters to check if filtered angles converged during initialisation
#define INIT_ANGLE_DIFFERENCE 0.5 // maximum angle difference between filtered angles and accelerometer angles (x- and y-axis) after initialisation
#define INIT_RATE 0.05            // maximum angular rate of Madgwick-filtered angle (z-axis) after initialisation
#define INIT_VELOCITY_V 0.10      // maximum vertical velocity after initialisation

// flight setpoint limits
#define ROLL_PITCH_THROTTLE_LIMIT 350
#define ROLL_PITCH_THROTTLE_ITERM_LIMIT 150 // 100
#define YAW_THROTTLE_LIMIT 250
#define YAW_THROTTLE_ITERM_LIMIT 200    	// 150
#define ROLL_PITCH_ANGLE_LIMIT 33           // deg   // 32, 30, 35
#define YAW_RATE_LIMIT 150                  // deg/s // 120, 180
#define VELOCITY_V_LIMIT 2.5                // m/s
#define VELOCITY_XY_LIMIT 1.5               // m/s   // 2.5 // TODO: Increase this to 5.0 m/s later.

// Throttle to enter started state and begin PID calculations.
// To ensure a smooth start, this value should be close to the throttle necessary for take off.
// An accurate value close to hover throttle also helps when switching flight modes from Stabilize to AltitudeHold or ReturnToLaunch.
#define THROTTLE_HOVER 1350
// Set throttle limit (< 2000), so there is some headroom for pid control in order to keep the quadcopter stable during full throttle.
#define THROTTLE_LIMIT 1725

// throttle deadzone (altitude hold) in per cent of throttle range
#define THROTTLE_DEADZONE_PCT 20

// minimum time in microseconds for no sensor update in order to set an error code
#define SENSOR_DT_LIMIT 2000000

// minimum time in microseconds the motion or control failsafe conditions need to be met in order to enter failsafe
#define FS_MOTION_CONTROL_DT_LIMIT 500000

// minimum time in microseconds for no imu update in order to enter failsafe
#define FS_IMU_DT_LIMIT 2000000

// failsafe motion limits
#define FS_MOTION_ANGLE_LIMIT 60
#define FS_MOTION_RATE_LIMIT 450

// failsafe control limits
#define FS_CONTROL_ANGLE_DIFF 30
#define FS_CONTROL_RATE_DIFF 120

// throttle deadzone limits within which altitude hold is active
const uint16_t THROTTLE_DEADZONE_BOT = 1000 + 10 * (50 - 0.5 * THROTTLE_DEADZONE_PCT);
const uint16_t THROTTLE_DEADZONE_TOP = 1000 + 10 * (50 + 0.5 * THROTTLE_DEADZONE_PCT);

// angle controller acceleration limits (deg/ss)
const float ACCEL_ROLL_PITCH_LIMIT = 720; // 1100, 720
const float ACCEL_YAW_LIMIT = 270;        // 270, 180

// angle time constants
const float TC_ROLL_PITCH_ANGLE = 0.15; // 0.15
const float TC_YAW_ANGLE = 0.15;

// vertical acceleration limit (m/ss)
const float ACCEL_V_LIMIT = 1.5; // 2.5
// horizontal acceleration limit (m/ss)
const float ACCEL_H_LIMIT = 1;
// altitude time constant
const float TC_ALTITUDE = 1;
// distance time constant
const float TC_DISTANCE = 2;

// angular rate PID values
const float P_ROLL_RATE = 3.2, I_ROLL_RATE = 7.0, D_ROLL_RATE = 0.013;    // 3.500, 7.000, 0.013; 4.000, 10.000, 0.015; 4.000, 12.000, 0.015; 5.0, 15.0, 0.020
const float P_PITCH_RATE = 3.2, I_PITCH_RATE = 7.0, D_PITCH_RATE = 0.013; // 3.500, 7.000, 0.013; 4.000, 10.000, 0.015; 4.000, 12.000, 0.015; 5.0, 15.0, 0.020
const float P_YAW_RATE = 3.5, I_YAW_RATE = 6.0, D_YAW_RATE = 0.000;       // 4.000, 8.000, 0.000; 3.000, 6.000, 0.000; 4.0, 10.0, 0.000

// vertical velocity PID values for altitude hold
const float P_VELOCITY_V = 55.0, I_VELOCITY_V = 12.0, D_VELOCITY_V = 0.000; // 60.0, 12.0, 0.000; 40.0, 10.0, 0.000;

// horizontal velocity PID values for return to launch
const float P_VELOCITY_H = 2.0, I_VELOCITY_H = 0.5, D_VELOCITY_H = 0.000; // 3.0, 0,7, 0,000;

// Cut of frequency f_c: https://dsp.stackexchange.com/questions/40462/exponential-moving-average-cut-off-frequency) (F_s = 9000)
// EMA = 0.0139 --> f_c = 20 Hz; EMA = 0.0035 --> f_c = 5 Hz;
// EMA = 0.1301 --> f_c = 200 Hz; EMA = 0.0674 --> f_c = 100 Hz;

// EMA filter parameter for accelerometer
const float EMA_ACCEL = 0.0003; // too high values will result in incorrect angle estimation within Madgwick filter

// EMA filter parameters for rates and proportional rate controller inputs.
const float EMA_ROLL_RATE = 0.030;  // 0.030
const float EMA_PITCH_RATE = 0.030; // 0.030
const float EMA_YAW_RATE = 0.010;   // 0.010
// EMA filter parameters for derivative rate controller inputs
const float EMA_ROLL_RATE_D = 0.005;  // 0.005
const float EMA_PITCH_RATE_D = 0.005; // 0.005
const float EMA_YAW_RATE_D = 0.005;   // 0.005

// EMA filter parameter for altitude
const float EMA_ALT = 0.0004;

// EMA filter parameter for vertical velocity and proportional vertical velocity controller input.
const float EMA_VELOCITY_V = 0.005;
// EMA filter parameter for derivative vertical velocity controller input.
const float EMA_VELOCITY_V_D = 0.001;

// EMA filter parameter for horizontal velocity and proportional horizontal velocity controller input.
const float EMA_VELOCITY_H = 0.020;
// EMA filter parameter for derivative horizontal velocity controller input.
const float EMA_VELOCITY_H_D = 0.005;

// EMA filter parameter for heading correction.
// This filter needs to be very strong, so the heading correction used to compensate inaccurate horizontal movement,
// caused by bad compass measurements and wind, is applied slowly.
const float EMA_HEADING_CORRECTION = 0.00005; // TODO: Test this parameter.

// the altitude for returning to launch is calculated by adding this offset to the maximum altitude reached during flight
const float RTL_RETURN_OFFSET = 0; // TODO: Add some offset when tested.
// the altitude for descending after returning to launch is calculated by adding this offset to the initial altitude
const float RTL_DESCEND_OFFSET = 5;

// failsafe configuration
const uint8_t FS_IMU = 0b00000001;
const uint8_t FS_MOTION = 0b00000010;
const uint8_t FS_CONTROL = 0b00000100;

// configure failsafe
const uint8_t FS_CONFIG = 0b00000011; // imu and motion failsafe

// list of error codes
const uint8_t ERROR_IMU   = 0b00000001;
const uint8_t ERROR_MAG   = 0b00000010;
const uint8_t ERROR_BAR   = 0b00000100;
const uint8_t ERROR_GPS   = 0b00001000;
const uint8_t ERROR_SDLOG = 0b00010000;

// state of initial estimation
enum class state { init, busy };

// estimate initial pose
bool initPose(float beta_init, float beta, float init_angleDifference, float init_rate);

// calculate accelerometer x and y angles in degrees
void accelAngles(float &roll_angle_accel, float &pitch_angle_accel);

// estimate initial altitude
bool initAltitude(float init_velocity_v);

// update LED according to quadcopter status (error, motors armed/disarmed and initialisation)
void updateLedStatus();

// calculate loop time in microseconds (dt) and seconds (dt_s)
void loopTime();

#ifdef USE_MAG
// get magnetometer data
void getMagData(int16_t &mx, int16_t &my, int16_t &mz);
#endif

#ifdef USE_BAR
// get barometer data
bool getBaroData(float &baroAltRaw);
#endif

#ifdef USE_GPS
// get gps data
void getGpsData(NeoGPS::Location_t &launch_location, NeoGPS::Location_t &current_location, float &heading, float &velocity_north, float &velocity_east);
#endif

// initialise quadcopter (pose, altitude) after first run or calibration
bool initQuad(bool &initialiseQuad);

// arm/disarm on rc command and disarm on failsafe conditions
void arm_failsafe(uint8_t fs_config);

// disarm and reset quadcopter
void disarmAndResetQuad();

// calculate roll and pitch angle setpoints as well as yaw rate setpoint from radio control input
void rc_rpAngle_yRate(float &roll_angle_sp, float &pitch_angle_sp, float &yaw_rate_sp);

#ifdef USE_GPS
// calculate xyv-velocity setpoints and yaw rate setpoint for returning to launch
void rtl_xyVelocity_yRate(float &velocity_x_sp, float &velocity_y_sp, float &velocity_v_sp, float &yaw_rate_sp);
#endif

#ifdef PLOT
// add time graphs to plot through Processing
void addTimeGraphs(Plotter &p);
#endif

#endif