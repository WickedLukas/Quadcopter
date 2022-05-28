//=====================================================================================================
// KalmanFilter1D.h
//=====================================================================================================
//
// One dimensional Kalman-Filter for accurate position in one dimension using acceleration and position
// measurements.
//
// A Comparison of Complementary and Kalman Filtering
// WALTER T. HIGGINS, JR.
//
//=====================================================================================================

#ifndef KALMANFILTER1D_H_
#define KALMANFILTER1D_H_

class KalmanFilter1D {

public:
	// constructor
	KalmanFilter1D(float timeConstant = 0.9); // timeConstant < 1: trust the acceleration measurement more
											  // timeConstant > 1: trust the position measurement more

	// update filter using acceleration and position measurements
	void update(float accelMeas, float posMeas, float dt_s);

	// get filter results
	float get_position();
	float get_velocity();

	// reset filter
	void reset(void);

private:
	// filter parameters
	float m_timeConstant;
	float k1, k2;

	// accurate postion and velocity (filter result)
	double position = 0;
	double velocity = 0;
};

#endif