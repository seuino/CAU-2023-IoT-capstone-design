#ifndef _DCMOTOR_h
#define _DCMOTOR_h

#include "Arduino.h"

extern const float TICKS_PER_METER;
extern const float TICKS_PER_RADIAN;

// .h
//////////////////////////////////////////////////
class DCMotor {
  public:
    DCMotor(uint8_t motorA_pin, uint8_t motorB_pin, uint8_t enable_pin, float K_B1, float K_B2);

    void controlPwm(int16_t pwm);
    int16_t calcPid(const float target_velocity);
    int16_t calcLinear(const float target_velocity);
    void calcVel();

    volatile int16_t encoder_ticks = 0;
    float velocity; //m/s
    // float angular_velocity; //rad/s

  private:
    //controlPwm
    uint8_t motorA_pin_;
    uint8_t motorB_pin_;
    uint8_t enable_pin_;

    //calcPid
    uint32_t previous_micros_;
    float pre_error_ = 0;
    float acc_error_ = 0;
    const float K_P_ = 2500;
    const float K_I_ = 100;
    const float K_D_ = 0;

    //calcLinear
    // y = K_B1 + K_B2 * x
    float K_B1_;
    float K_B2_;

    //calcVel
    uint32_t previous_millis_ = 0;
    int16_t previous_ticks_ = 0;

};
#endif
