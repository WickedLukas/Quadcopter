#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>

// TODO: Tune parameters for altitude hold mode (barometer filter settings, PID, ...).

// define which sensors shall be used
#define USE_BAR // use barometer
#define USE_MAG // use magnetometer // TODO: Check if magnetometer data is reliable when motors are running.
#define USE_GPS // use GPS

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
#define BAROMETER_INTERRUPT_PIN 20

// pwm pins to control motors
// 1: top-left (CW); 2: top-right (CCW); 3: bottom-left (CW); 4: bottom-right (CCW);
#define MOTOR_PIN_1 2
#define MOTOR_PIN_2 3
#define MOTOR_PIN_3 4
#define MOTOR_PIN_4 5

// motor pwm resolution
#define MOTOR_PWM_RESOLUTION 11

// motor pwm frequency
#define MOTOR_PWM_FREQUENCY 4000

#define BETA_INIT 10 // Madgwick algorithm gain (2 * proportional gain (Kp)) during initial pose estimation - 10
#define BETA 0.033   // Madgwick algorithm gain (2 * proportional gain (Kp)) - 0.041 MARG, 0.033 IMU

// time constant for altitude filter
// TC < 1: trust the acceleration measurement more
// TC > 1: trust the position measurement more
#define TC_ALTITUDE_FILTER 0.9 // 0.9

// parameters to check if filtered angles converged during initialisation
#define INIT_ANGLE_DIFFERENCE 0.5 // maximum angle difference between filtered angles and accelerometer angles (x- and y-axis) after initialisation
#define INIT_RATE 0.05            // maximum angular rate of Madgwick-filtered angle (z-axis) after initialisation
#define INIT_VELOCITY_V 0.20      // maximum vertical velocity after initialisation

// flight setpoint limits
#define YAW_RATE_LIMIT 120 // deg/s

#define ROLL_ANGLE_LIMIT 30  // deg
#define PITCH_ANGLE_LIMIT 30 // deg

#define VELOCITY_V_LIMIT 2.5 // m/s

// throttle when armed (slightly above esc/motor deadzone)
#define THROTTLE_ARMED 1125
// Throttle to enter started state and begin PID calculations.
// The throttle stick position is centered around this value.
// To ensure a smooth start, this value should be close to the throttle necessary for take off.
#define THROTTLE_HOVER 1475
// Set throttle limit (< 2000), so there is some headroom for pid control in order to keep the quadcopter stable during full throttle.
#define THROTTLE_LIMIT 1800

// throttle deadzone (altitude hold) in per cent of throttle range
#define THROTTLE_DEADZONE_PCT 20

// sensor update time limit
#define SENSOR_DT_LIMIT 2000000

// minimum time in microseconds a failsafe condition needs to be met, in order to activate it (except for FS_IMU)
#define FS_TIME 500000

// failsafe imu update time limit
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
//const float ACCEL_MIN_ROLL_PITCH = 40;
//const float ACCEL_MIN_YAW = 10;
const float ACCEL_MAX_ROLL_PITCH = 1100; // 1100, 720
const float ACCEL_MAX_YAW = 180;         // 270, 180
// angle time constant
const float TC_ANGLE = 0.15;

// throttle expo parameter used to achieve less throttle sensitivity around hover
const float THROTTLE_EXPO = 0.3; // 0.3

// vertical acceleration limit (m/ss)
const float ACCEL_V_MAX = 1;
// altitude time constant
const float TC_ALTITUDE = 2;

// angular rate PID values
const float P_ROLL_RATE = 2.000, I_ROLL_RATE = 0.000, D_ROLL_RATE = 0.020;    // 2.500, 0.000, 0.023 @ 0.006 EMA_RATE
const float P_PITCH_RATE = 2.000, I_PITCH_RATE = 0.000, D_PITCH_RATE = 0.020; // 2.500, 0.000, 0.023 @ 0.006 EMA_RATE
const float P_YAW_RATE = 3.500, I_YAW_RATE = 2.000, D_YAW_RATE = 0.000;       // 3.500, 2.000, 0.000

// vertical velocity PID values for altitude hold
const float P_VELOCITY_V = 250.000, I_VELOCITY_V = 5.000, D_VELOCITY_V = 0.000; // 250.000, 5.000, 0.000

// Cut of frequency f_c: https://dsp.stackexchange.com/questions/40462/exponential-moving-average-cut-off-frequency)
// EMA filter parameters for proportional (P) and derivative (D) rate controller inputs.
// EMA = 0.1301 --> f_c = 200 Hz; EMA = 0.0674 --> f_c = 100 Hz;
const float EMA_ROLL_RATE_P = 0.040;  // 0.040
const float EMA_PITCH_RATE_P = 0.040; // 0.040
const float EMA_YAW_RATE_P = 0.020;   // 0.020
// EMA = 0.0139 --> f_c = 20 Hz; EMA = 0.0035 --> f_c = 5 Hz;
const float EMA_ROLL_RATE_D = 0.005;  // 0.005
const float EMA_PITCH_RATE_D = 0.005; // 0.005
const float EMA_YAW_RATE_D = 0.005;   // 0.005

// EMA filter parameters for proportional (P)  and derivative (D) vertical velocity controller inputs.
// EMA = 0.1301 --> f_c = 200 Hz; EMA = 0.0674 --> f_c = 100 Hz;
const float EMA_velocity_v_P = 0.015; // 0.015
// EMA = 0.0139 --> f_c = 20 Hz; EMA = 0.0035 --> f_c = 5 Hz;
const float EMA_velocity_v_D = 0.005; // 0.005

// failsafe configuration
const uint8_t FS_IMU = 0b00000001;
const uint8_t FS_MOTION = 0b00000010;
const uint8_t FS_CONTROL = 0b00000100;

// configure failsafe
const uint8_t FS_CONFIG = 0b00000011; // imu and motion failsafe

// list of error codes
const uint8_t ERROR_IMU = 0b00000001;
const uint8_t ERROR_MAG = 0b00000010;
const uint8_t ERROR_BAR = 0b00000100;
const uint8_t ERROR_GPS = 0b00001000;

// factors for converting between radians and degrees
const float RAD2DEG = (float)4068 / 71;
const float DEG2RAD = (float)71 / 4068;

// state of initial estimation
enum class state { init, busy };

// estimate initial pose
bool initPose(float beta_init, float beta, float init_angleDifference, float init_rate);

// calculate accelerometer x and y angles in degrees
void accelAngles(float &roll_angle_accel, float &pitch_angle_accel);

// estimate initial altitude
bool initAltitude(float init_velocity_v);

// calculate acceleration in ned-frame relative to gravity using acceleration in sensor-frame and pose
void calc_accel_ned_rel(float &a_n_rel, float &a_e_rel, float &a_d_rel);

// multiply two quaternions (Hamilton product)
void qMultiply(float *q1, float *q2, float *result_q);

// arm/disarm on rc command and disarm on failsafe conditions
void arm_failsafe(uint8_t fs_config);

// disarm and reset quadcopter
void disarmAndResetQuad();

#endif