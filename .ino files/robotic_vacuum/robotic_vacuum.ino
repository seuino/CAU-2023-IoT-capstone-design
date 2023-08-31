#define USE_USBCON

// #define SERIAL_DEBUG
#ifdef SERIAL_DEBUG
  #define BAUDRATE 115200
#endif

// dcmotor
//////////////////////////////////////////////////
#define ENCODER_1A_PIN 22
#define ENCODER_1B_PIN 24
#define MOTOR_1A_PIN 26
#define MOTOR_1B_PIN 28
#define ENABLE_1_PIN 2

#define ENCODER_2A_PIN 23
#define ENCODER_2B_PIN 25
#define MOTOR_2A_PIN 27
#define MOTOR_2B_PIN 29
#define ENABLE_2_PIN 3

#define MOTOR_3A_PIN 30
#define MOTOR_3B_PIN 32
#define ENABLE_3_PIN 4

#define MOTOR_4A_PIN 31
#define MOTOR_4B_PIN 33
#define ENABLE_4_PIN 5

#define GEAR_RATIO 210.59
#define CPR 12
#define WHEEL_DIA 60 //mm
#define INTERMOTOR_DIST 115 //mm

#define TICKS_PER_DEGREE GEAR_RATIO*CPR/360
#define TICKS_PER_METER GEAR_RATIO*CPR/WHEEL_DIA/PI

#include "DCMotor.h"

DCMotor dcMotor1(MOTOR_1A_PIN, MOTOR_1B_PIN, ENABLE_1_PIN);
DCMotor dcMotor2(MOTOR_2A_PIN, MOTOR_2B_PIN, ENABLE_2_PIN);

DCMotor pump1(MOTOR_3A_PIN, MOTOR_3B_PIN, ENABLE_3_PIN);
DCMotor pump2(MOTOR_4A_PIN, MOTOR_4B_PIN, ENABLE_4_PIN);

void doEncoder_1A() {dcMotor1.encoder_ticks += (digitalRead(ENCODER_1A_PIN)==digitalRead(ENCODER_1B_PIN))?1:-1;}
void doEncoder_1B() {dcMotor1.encoder_ticks += (digitalRead(ENCODER_1A_PIN)==digitalRead(ENCODER_1B_PIN))?-1:1;}
void doEncoder_2A() {dcMotor2.encoder_ticks += (digitalRead(ENCODER_2A_PIN)==digitalRead(ENCODER_2B_PIN))?1:-1;}
void doEncoder_2B() {dcMotor2.encoder_ticks += (digitalRead(ENCODER_2A_PIN)==digitalRead(ENCODER_2B_PIN))?-1:1;}

// servo
//////////////////////////////////////////////////
#define SERVO_1_PIN 6
#define SERVO_2_PIN 7

#include <Servo.h>

Servo servo1;
Servo servo2;

// rosserial
//////////////////////////////////////////////////
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;

std_msgs::Int32 left_ticks_msg;
std_msgs::Int32 right_ticks_msg;
ros::Publisher left_ticks("/left_ticks", &left_ticks_msg);
ros::Publisher right_ticks("/right_ticks", &right_ticks_msg);

float linear_x = 0;
float angular_z = 0;

void getCmdVel(const geometry_msgs::Twist &data) {
  linear_x = data.linear.x;
  angular_z = data.angular.z;
}
ros::Subscriber <geometry_msgs::Twist> cmd_vel("/cmd_vel", &getCmdVel);


void setup() {
  #ifdef SERIAL_DEBUG
    Serial.begin(BAUDRATE);
  #endif

  // dcmotor
  pinMode(ENCODER_1A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_1B_PIN, INPUT_PULLUP);
  pinMode(MOTOR_1A_PIN, OUTPUT);
  pinMode(MOTOR_1B_PIN, OUTPUT);
  pinMode(ENABLE_1_PIN, OUTPUT);
  pinMode(ENCODER_2A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_2B_PIN, INPUT_PULLUP);
  pinMode(MOTOR_2A_PIN, OUTPUT);
  pinMode(MOTOR_2B_PIN, OUTPUT);
  pinMode(ENABLE_2_PIN, OUTPUT);

  pinMode(MOTOR_3A_PIN, OUTPUT);
  pinMode(MOTOR_3B_PIN, OUTPUT);
  pinMode(ENABLE_3_PIN, OUTPUT);
  pinMode(MOTOR_4A_PIN, OUTPUT);
  pinMode(MOTOR_4B_PIN, OUTPUT);
  pinMode(ENABLE_4_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_1A_PIN), doEncoder_1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_1B_PIN), doEncoder_1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2A_PIN), doEncoder_2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2B_PIN), doEncoder_2B, CHANGE);

  // servo
  servo1.attach(SERVO_1_PIN);
  servo2.attach(SERVO_2_PIN);

  // rosserial
  nh.initNode();
  nh.advertise(left_ticks);
  nh.advertise(right_ticks);
  nh.subscribe(cmd_vel);
}

void loop() {
  // For motor control
  // dcMotor1.control_pwm(1, 255);
  // dcMotor2.control_pwm(1, 255);
  achieveCmdVel();
  // pump1.control_pwm(1, 255);
  // pump2.control_pwm(1, 255);
  // servo1.write(0);
  // servo1.write(0);
  
  // For monitoring in ros
  left_ticks_msg.data = dcMotor1.encoder_ticks;
  right_ticks_msg.data = dcMotor2.encoder_ticks;
  left_ticks.publish(&left_ticks_msg);
  right_ticks.publish(&right_ticks_msg);
  nh.spinOnce();
  delay(10);

  #ifdef SERIAL_DEBUG
    Serial.print(dcMotor1.encoder_ticks);
    Serial.print(",");
    Serial.print(dcMotor2.encoder_ticks);
    Serial.println();
  #endif

}

void achieveCmdVel() {
  if(angular_z > 0) { //회전 우선
    dcMotor1.control_pwm(2,255);
    dcMotor2.control_pwm(1,255);
  }
  else if(angular_z < 0) {
    dcMotor1.control_pwm(1,255);
    dcMotor2.control_pwm(2,255);
  }
  else if(linear_x > 0) {
    dcMotor1.control_pwm(1,255);
    dcMotor2.control_pwm(1,255);
  }
  else if(linear_x < 0) {
    dcMotor1.control_pwm(2,255);
    dcMotor2.control_pwm(2,255);
  }
  else {
    dcMotor1.control_pwm(3,255);
    dcMotor2.control_pwm(3,255);
  }
}