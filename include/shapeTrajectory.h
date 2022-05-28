#ifndef SHAPETRAJECTORY_H_
#define SHAPETRAJECTORY_H_

// map input from [in_min, in_max] to [out_min, out_max] and in_between to out_between
float map3(float in, float in_min, float in_between, float in_max, float out_min, float out_between, float out_max);

// Generate exponential (cubic) curve.
// x = [-1, 1]; expo = [0, 1];
float expo_curve(float x, float expo);

// Calculate the velocity correction from the position error. The velocity has acceleration and deceleration limits including a basic jerk limit using timeConstant.
float shape_position(float position_error, float timeConstant, float accel_max, float last_velocity_sp, float dt_s);

// proportional controller with sqrt sections to constrain the acceleration
float sqrtController(float position_error, float p, float accel_max, float dt_s);

// limit the acceleration/deceleration of a velocity request
float shape_velocity(float desired_velocity_sp, float accel_max, float last_velocity_sp, float dt_s);

#endif