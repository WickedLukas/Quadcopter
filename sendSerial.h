/*
 * sendSerial.h
 *
 * Created: 06.07.2019 14:48:46
 *  Author: Lukas
 */ 

#ifndef SENDSERIAL_H_
#define SENDSERIAL_H_

#include <Arduino.h>

// send serial data to Processing
void sendSerial(Stream &port, float dt, float angle_x, float angle_y, float angle_z);

#endif /* SENDSERIAL_H_ */