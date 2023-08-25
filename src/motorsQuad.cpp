#include "motorsQuad.h"
#include "common.h"

#include <Arduino.h>

// constructor
MotorsQuad::MotorsQuad(uint8_t motor1_pin, uint8_t motor2_pin, uint8_t motor3_pin, uint8_t motor4_pin, uint16_t motor_pwm_frequency,
                       float thrust_min_ratio, float thrust_max_ratio, float thrust_expo) {
    // initialise member variables
    m_motor1_pin = motor1_pin;
    m_motor2_pin = motor2_pin;
    m_motor3_pin = motor3_pin;
    m_motor4_pin = motor4_pin;

    m_motor_pwm_frequency = motor_pwm_frequency;

    m_thrust_min_ratio = constrain(thrust_min_ratio, 0.f, 1.f);
    m_thrust_max_ratio = constrain(thrust_max_ratio, 0.f, 1.f);
    m_thrust_expo = constrain(thrust_expo, -1.f, 1.f);

    m_state = State::disarmed;

    // initialise motor pins
    init_pin(m_motor1_pin);
    init_pin(m_motor2_pin);
    init_pin(m_motor3_pin);
    init_pin(m_motor4_pin);
}

void MotorsQuad::output(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4) {
    pwm1 = lineariseMotor(pwm1);
    pwm2 = lineariseMotor(pwm2);
    pwm3 = lineariseMotor(pwm3);
    pwm4 = lineariseMotor(pwm4);

    noInterrupts();
    m_oldResolution = analogWriteResolution(11);

    switch (m_state) {
    case State::armed:
        analogWriteMotors(pwm1, pwm2, pwm3, pwm4);
        break;

    case State::disarmed:
        analogWriteMotors(0, 0, 0, 0);
        break;

    case State::arming:
        if (micros() - t0_arming > ARMING_DISARMING_TIME)
        {
            m_state = State::armed;
            DEBUG_PRINTLN(F("Armed!"));
        }
        else
        {
            analogWriteMotors(1000, 1000, 1000, 1000);
        }
        break;

    case State::disarming:
        if (micros() - t0_disarming > ARMING_DISARMING_TIME)
        {
            m_state = State::disarmed;
            DEBUG_PRINTLN(F("Disarmed!"));
        }
        else
        {
            analogWriteMotors(1000, 1000, 1000, 1000);
        }
        break;
    }

    analogWriteResolution(m_oldResolution);
    interrupts();
}

void MotorsQuad::arm() {
    // arming is only possible when disarmed and no error has occurred
    if ((m_state == State::disarmed) && (error_code == 0)) {
        t0_arming = micros();
        m_state = State::arming;
    }
}

void MotorsQuad::disarm() {
    // disarming when armed or arming (disarm will cancel arming process)
    if ((m_state == State::armed) || (m_state == State::arming)) {
        t0_disarming = micros();
        m_state = State::disarming;
    }
}

MotorsQuad::State MotorsQuad::getState() {
    return m_state;
}

// initialise motor output pin
void MotorsQuad::init_pin(uint8_t pin) {
    analogWriteFrequency(pin, m_motor_pwm_frequency);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

// linearise motor output for linear thrust response
uint16_t MotorsQuad::lineariseMotor(uint16_t pwm) {
    float thrust = (constrain(pwm, 1000, 2000) - 1000) * 0.001f;

    // apply thrust curve
    if (m_thrust_expo != 0) // zero expo means linear
    {
        thrust = ((m_thrust_expo - 1) + sqrt(pow((1 - m_thrust_expo), 2) + 4 * m_thrust_expo * thrust)) / (2 * m_thrust_expo);
    }

    // apply thrust limits
    thrust = m_thrust_min_ratio + (m_thrust_max_ratio - m_thrust_min_ratio) * thrust;

    pwm = (uint16_t) ((1000 + 1000 * thrust) + 0.5);

    return constrain(pwm, 1000, 2000);
}

// analog write to all motors
void MotorsQuad::analogWriteMotors(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4) {
#ifndef MOTORS_OFF // for safety, MOTORS_OFF can be defined to prevent motors from running
    analogWrite(m_motor1_pin, pwm1);
    analogWrite(m_motor2_pin, pwm2);
    analogWrite(m_motor3_pin, pwm3);
    analogWrite(m_motor4_pin, pwm4);
#endif
}