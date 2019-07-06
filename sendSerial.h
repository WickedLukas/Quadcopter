/*
 * sendSerial.h
 *
 * Created: 06.07.2019 14:48:46
 *  Author: Lukas
 */ 


#ifndef SENDSERIAL_H_
#define SENDSERIAL_H_

// send serial data to Processing
void sendSerial(float angle_x_gyro, float angle_y_gyro, float angle_z_gyro, float angle_x_accel, float angle_y_accel, float angle_x_CF, float angle_y_CF, float angle_x_KF, float angle_y_KF) {
    // send dt, accelerometer angles, gyro angles and filtered angles
    //Serial.print(F("DEL:"));
    //Serial.print(dt, DEC);
    //Serial.print(F("#GYR:"));
    //Serial.print(angle_x_gyro, 2); Serial.print(F(",")); Serial.print(angle_y_gyro, 2);	Serial.print(F(",")); Serial.print(angle_z_gyro, 2);
    //Serial.print(F("#ACC:"));
    //Serial.print(angle_x_accel, 2);	Serial.print(F(",")); Serial.print(angle_y_accel, 2);
    //Serial.print(F("#CFI:"));
    //Serial.print(angle_x_CF, 2); Serial.print(F(",")); Serial.print(angle_y_CF, 2);
    //Serial.print(F("#KFI:"));
    //Serial.print(angle_x_KF, 2); Serial.print(F(",")); Serial.print(angle_y_KF, 2);
    //Serial.println(F(""));
}

#endif /* SENDSERIAL_H_ */