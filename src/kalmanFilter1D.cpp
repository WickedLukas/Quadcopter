//=====================================================================================================
// KalmanFilter1D.cpp
//=====================================================================================================
//
// One dimensional Kalman-Filter for accurate position in one dimension using acceleration and position 
// measurements.
// 
// A Comparison of Complementary and Kalman Filtering
// WALTER T. HIGGINS, JR.
//
//=====================================================================================================

#include "kalmanFilter1D.h"

#include <math.h>

// constructor
KalmanFilter1D::KalmanFilter1D(float timeConstant) {
	m_timeConstant = timeConstant;
	
	k1 = sqrt(2 * m_timeConstant);
	k2 = m_timeConstant;
}

// update filter using acceleration and position measurements
void KalmanFilter1D::update(float accelMeas, float posMeas, float dt_s) {
	position += (double) velocity * dt_s + (double) (posMeas - position) * (k1 + 0.5 * dt_s * k2) * dt_s + (double) accelMeas * 0.5 * dt_s * dt_s;
	velocity += (double) (posMeas - position) * k2 * dt_s + (double) accelMeas * dt_s;
}

float KalmanFilter1D::get_position(void) {
	return position;
}

float KalmanFilter1D::get_velocity(void) {
	return velocity;
}

// reset filter
void KalmanFilter1D::reset(void) {
	position = 0;
	velocity = 0;
}