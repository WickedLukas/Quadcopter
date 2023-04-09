#include "common.h"

#include <core_pins.h>

// update LED
// mode: 0		off
// mode: 1		blink
// mode: > 1	on
void updateLed(uint8_t pin, uint8_t mode, uint32_t interval_ms) {
	static uint8_t ledState = LOW;
	static uint32_t t0_ms = 0, t_ms = 0;
	
	if (mode == 0) {
		ledState = LOW;
	}
	else if (mode == 1) {
		t_ms = millis();
		
		if ((t_ms - t0_ms) > interval_ms) {
		t0_ms = t_ms;
			
			if (ledState == LOW) {
				ledState = HIGH;
			}
			else {
				ledState = LOW;
			}
		}
	}
	else {
		ledState = HIGH;
	}
	
	// set LED state
	digitalWrite(pin, ledState);
}

// adjust angle range to [min, max)
void adjustAngleRange(float min, float max, float &angle)
{
	if (angle >= max) {
		angle -= (max - min);
	}
	else if (angle < min) {
		angle += (max - min);
	}
}

// transform quaternion from body- to ned-frame using the pose quaternion
void body2nedFrame(const float *pose_q, const float *body_q, float *ned_q) {
	static float helper_q[4];
	// conjugate complex pose quaternion, which is equal to its inverse, since it is a unit quaternion
	static float pose_q_conj[4];
	pose_q_conj[0] = pose_q[0]; pose_q_conj[1] = -pose_q[1]; pose_q_conj[2] = -pose_q[2]; pose_q_conj[3] = -pose_q[3];

	// calculate the ned-quaternion by rotating the body-quaternion with the pose-quaternion (unit quaternion): ned_q = pose_q * body_q * pose_q_conj
	qMultiply(pose_q, body_q, helper_q);
	qMultiply(helper_q, pose_q_conj, ned_q);
}

// multiply two quaternions (Hamilton product)
void qMultiply(const float *q1, const float *q2, float *q_result) {
	q_result[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
	q_result[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
	q_result[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
	q_result[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
}