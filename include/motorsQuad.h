#ifndef MOTORSQUAD_H_
#define MOTORSQUAD_H_

#include <stdint.h>

// motor class
class MotorsQuad {

public:
    enum class State { armed, disarmed, arming, disarming };

    // constructor
    MotorsQuad(uint8_t motor1_pin, uint8_t motor2_pin, uint8_t motor3_pin, uint8_t motor4_pin, uint8_t motor_pwm_resolution, uint16_t motor_pwm_frequency);

    void output(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4);
    void arm();
    void disarm();
    State getState();

private:
    void init_pin(uint8_t pin); // initialise motor output pin

    void analogWriteMotors(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4); // analogWrite to all motors

    // arming and disarming time in microseconds
    const uint32_t ARMING_DISARMING_TIME = 2000000;

    uint8_t m_motor1_pin, m_motor2_pin, m_motor3_pin, m_motor4_pin;
    uint8_t m_motor_pwm_resolution;
    uint16_t m_motor_pwm_frequency;

    // motor state
    State m_state = State::disarmed;

    uint32_t t0_arming, t0_disarming;

    uint8_t m_oldResolution;
};

#endif