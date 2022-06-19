#include "shapeTrajectory.h"

#include <wiring.h>
#include <math.h>

// map input from [in_min, in_max] to [out_min, out_max] and in_between to out_between
float map3(float in, float in_min, float in_between, float in_max, float out_min, float out_between, float out_max) {
	if (in < in_between) {
		return (map(in, in_min, in_between, out_min, out_between));
	}
	else {
		return (map(in, in_between, in_max, out_between, out_max));
	}
}

// Generate exponential (cubic) curve.
// x = [-1, 1]; expo = [0, 1];
float expo_curve(float x, float expo) {
	return (1.0f - expo) * x + expo * x * x * x;
}

// Calculate the velocity correction from the position error. The velocity has acceleration and deceleration limits including a basic jerk limit using timeConstant.
float shape_position(float position_error, float timeConstant, float accel_max, float last_velocity_sp, float dt_s) {
	static float desired_velocity_sp;
	
	// calculate the velocity as position_error approaches zero with acceleration limited by accel_max
	desired_velocity_sp = sqrtController(position_error, 1.0 / max(timeConstant, 0.01), accel_max, dt_s);
	
	// acceleration is limited directly to smooth the beginning of the curve
	return shape_velocity(desired_velocity_sp, accel_max, last_velocity_sp, dt_s);
}

// proportional controller with sqrt sections to constrain the acceleration
float sqrtController(float position_error, float p, float accel_max, float dt_s) {
	static float correction_rate;
	static float linear_dist;
	
	linear_dist = accel_max / sq(p);
	
	if (position_error > linear_dist) {
		correction_rate = sqrt(2 * accel_max * (position_error - (linear_dist / 2)));
	}
	else if (position_error < -linear_dist) {
		correction_rate = -sqrt(2 * accel_max * (-position_error - (linear_dist / 2)));
	}
	else {
		correction_rate = position_error * p;
	}
	
	if (dt_s > 0.000010) {
		// this ensures we do not get small oscillations by over shooting the error correction in the last time step
		return constrain(correction_rate, -abs(position_error) / dt_s, abs(position_error) / dt_s);
	}
	else {
		return correction_rate;
	}
}

// limit the acceleration/deceleration of a velocity request
float shape_velocity(float desired_velocity_sp, float accel_max, float last_velocity_sp, float dt_s) {
	static float delta_velocity;
	
	delta_velocity = accel_max * dt_s;
	return constrain(desired_velocity_sp, last_velocity_sp - delta_velocity, last_velocity_sp + delta_velocity);
}