#include "DCMotor.h"

extern const float TICKS_PER_METER;
extern const float TICKS_PER_RADIAN;

// .cpp
//////////////////////////////////////////////////
DCMotor::DCMotor(uint8_t motorA_pin, uint8_t motorB_pin, uint8_t enable_pin) {

  motorA_pin_ = motorA_pin;
  motorB_pin_ = motorB_pin;
  enable_pin_ = enable_pin;

}
void DCMotor::controlPwm(int16_t pwm) {

  if(pwm > 0) {
    pwm = min(255, pwm); //max 255
    digitalWrite(motorA_pin_,HIGH);
    digitalWrite(motorB_pin_,LOW);
  }
  else if(pwm < 0){
    pwm = max(-255, pwm); //min -255
    digitalWrite(motorA_pin_,LOW);
    digitalWrite(motorB_pin_,HIGH);
  }
  else {
    digitalWrite(motorA_pin_,HIGH);
    digitalWrite(motorB_pin_,HIGH);
  }
  analogWrite(enable_pin_, pwm);


  // For monitoring in arduino
  // Serial.println(degree);

  // For monitoring in python

}
int16_t DCMotor::calcPid(const float target_velocity) { //run calcVel() first

  uint32_t current_micros = micros()
  static uint32_t previous_micros = current_micros - 1e-9;
  uint16_t diff_micros = current_micros - previous_micros;

  previous_micros = current_micros;

  float error = target_velocity - velocity;
  static float pre_error = error;
  float diff_error = error - pre_error;
  static float acc_error = 0;
  acc_error += error;

  pre_error = error;

  float pwm_p = K_P_ * error;
  float pwm_i = K_I_ * acc_error;
  float pwm_d = K_D_ * diff_error/(diff_micros*1e-6);
  float pwm_pid = pwm_p + pwm_i + pwm_d;

  return static_cast<int16_t>(pwm_pid);

}
uint16_t DCMotor::calcLinear(const float target_velocity) {
  
  float pwm_linear;
  if(target_velocity > 0) {
    pwm_linear = K_B1_ + K_B2_ * target_velocity;
  }
  else if(target_velocity < 0) {
    pwm_linear = -K_B1_ + K_B2_ * target_velocity;
  }
  else {
    pwm_linear = 0;
  }

  return static_cast<int16_t>(pwm_linear);

}
void DCMotor::calcVel() {
  
  static uint32_t previous_millis = 0;
  static int16_t previous_ticks = 0;
  int16_t current_ticks = encoder_ticks;
  uint32_t current_millis = millis();

  uint16_t cycle_ticks = current_ticks - previous_ticks;
  if (cycle_ticks > 10000) { //underflow
    cycle_ticks = -65536 + cycle_ticks; //((-32768) - previous_ticks) + (current_ticks - 32767) - 1
  }
  else if (cycle_ticks < -10000) { //overflow
    cycle_ticks = 65536 + cycle_ticks; //(32767 - previous_ticks) + (current_ticks - (-32768)) + 1
  }

  // angular_velocity = cycle_ticks/TICKS_PER_RADIAN/((current_millis-previous_millis)*1e-3);
  velocity = cycle_ticks/TICKS_PER_METER/((current_millis-previous_millis)*1e-3);
 
  previous_ticks = current_ticks;
  previous_millis = current_millis;
 
}