#ifndef MOTORSQUAD_H_
#define MOTORSQUAD_H_

#include <stdint.h>

// motor class
class MotorsQuad {

public:
    enum class State { armed, disarmed, arming, disarming };

    // constructor
    MotorsQuad(uint8_t motor1_pin, uint8_t motor2_pin, uint8_t motor3_pin, uint8_t motor4_pin, uint16_t motor_pwm_frequency,
                       float thrust_min_ratio = 0, float thrust_max_ratio = 1, float thrust_expo = 0);

    void output(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4);
    void arm();
    void disarm();
    State getState();

private:
    void init_pin(uint8_t pin);            // initialise motor output pin
    uint16_t lineariseMotor(uint16_t pwm); // linearise motor output for linear thrust response
    void analogWriteMotors(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4); // analog write to all motors

    // arming and disarming time in microseconds
    const uint32_t ARMING_DISARMING_TIME = 2000000;

    uint8_t m_motor1_pin, m_motor2_pin, m_motor3_pin, m_motor4_pin;
    uint16_t m_motor_pwm_frequency;

    // parameters for thrust linearisation
    float m_thrust_min_ratio; // ratio of the full throttle range which produces the minimum thrust [0, 1]
    float m_thrust_max_ratio; // ratio of the full throttle range which produces the maximum thrust [0, 1]
    float m_thrust_expo;      // set 0 for a linear and 1 for a second order thrust approximation [0, 1]

    // motor state
    State m_state = State::disarmed;

    uint32_t t0_arming, t0_disarming;

    uint8_t m_oldResolution;
};

#endif