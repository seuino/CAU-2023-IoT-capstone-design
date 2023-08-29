#include "TwoWheeledMobile.h"
// .cpp
//////////////////////////////////////////////////
DCMotor::DCMotor(uint8_t motorA_pin, uint8_t motorB_pin, uint8_t enable_pin) {

  _motorA_pin = motorA_pin;
  _motorB_pin = motorB_pin;
  _enable_pin = enable_pin;

}
void DCMotor::control_pwm(int key, uint8_t pwm) {

  analogWrite(_enable_pin, pwm);
  switch(key) {
    case 1:
      digitalWrite(_motorA_pin,HIGH);
      digitalWrite(_motorB_pin,LOW);
      break;
    case 2:
      digitalWrite(_motorA_pin,LOW);
      digitalWrite(_motorB_pin,HIGH);
      break;
    case 3:
      digitalWrite(_motorA_pin,HIGH);
      digitalWrite(_motorB_pin,HIGH);
      break;
  }

  // For monitoring in arduino
  // Serial.println(degree);

  // For monitoring in python

}