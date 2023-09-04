#ifndef _DCMOTOR_h
#define _DCMOTOR_h

#include "Arduino.h"
// .h
//////////////////////////////////////////////////
class DCMotor {
  public:
    DCMotor(uint8_t motorA_pin, uint8_t motorB_pin, uint8_t enable_pin);

    void controlPwm(int16_t pwm);
    int16_t calcPid(const float target_velocity);
    int16_t calcLinear(const float target_velocity);
    void calcVel();

    volatile int16_t encoder_ticks = 0;
    float velocity; //m/s
    // float angular_velocity; //rad/s

  private:
    uint8_t motorA_pin_;
    uint8_t motorB_pin_;
    uint8_t enable_pin_;

    const float K_P_ = 10;
    const float K_I_ = 10;
    const float K_D_ = 10;

    // y = K_B1 + K_B2 * x
    const float K_B1_ = 10;
    const float K_B2_ = 10;

};
#endif
