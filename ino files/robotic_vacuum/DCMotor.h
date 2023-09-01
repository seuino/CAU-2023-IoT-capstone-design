#ifndef _DCMOTOR_h
#define _DCMOTOR_h

#include "Arduino.h"
// .h
//////////////////////////////////////////////////
class DCMotor {
  public:
    DCMotor(uint8_t motorA_pin, uint8_t motorB_pin, uint8_t enable_pin);

    void control_pwm(int key, uint8_t pwm);

    volatile int32_t encoder_ticks = 0;
    float degree = 0;

  private:
    uint8_t _motorA_pin;
    uint8_t _motorB_pin;
    uint8_t _enable_pin;

};
#endif
