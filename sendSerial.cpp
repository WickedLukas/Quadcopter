/*
 * sendSerial.cpp
 *
 * Created: 06.07.2019 15:07:40
 *  Author: Lukas
 */ 

#include "sendSerial.h"

// send serial data to Processing
void sendSerial(Stream &port, float dt, float angle_x, float angle_y, float angle_z) {
    // send dt and angles
    port.print(F("DEL:"));
    port.print(dt, DEC);
    port.print(F("#MDF:"));
    port.print(angle_x, 2); port.print(F(",")); port.print(angle_y, 2); port.print(F(",")); port.print(angle_z, 2);
    port.println(F(""));
}