/*
 * sendSerial.cpp
 *
 * Created: 06.07.2019 15:07:40
 *	Author: Lukas
 */ 

#include "sendSerial.h"

// send serial data to Processing
void sendSerial(float dt, float angle_x, float angle_y, float angle_z) {
	// send dt and angles
	Serial.print(F("DEL:"));
	Serial.print(dt, DEC);
	Serial.print(F("#MDF:"));
	Serial.print(angle_x, 2); Serial.print(F(",")); Serial.print(angle_y, 2); Serial.print(F(",")); Serial.print(angle_z, 2);
	Serial.println(F(""));
}