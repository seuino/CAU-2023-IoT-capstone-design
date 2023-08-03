// serial
//////////////////////////////////////////////////
#define BAUDRATE 19200
// #define USE_USBCON

// dc motor
//////////////////////////////////////////////////
#include "DCMotor.h"

#define ENCODER_1A_PIN 2
#define ENCODER_1B_PIN 3
#define MOTOR_1A_PIN 4
#define MOTOR_1B_PIN 5
#define ENABLE_1_PIN 6

#define ENCODER_2A_PIN 7
#define ENCODER_2B_PIN 8
#define MOTOR_2A_PIN 9
#define MOTOR_2B_PIN 10
#define ENABLE_2_PIN 11

const int GEAR_RATIO = 100;
const int CPR = 12;
const int WHEEL_DIA = 60; //mm
const int INTERMOTOR_DIST = 155; //mm

const float TICKS_PER_DEGREE = GEAR_RATIO*CPR/360.0;
const float TICKS_PER_METER = GEAR_RATIO*CPR/(WHEEL_DIA/1000.0)/PI;

// sonar sensor
//////////////////////////////////////////////////
#include <NewPing.h>

#define TRIGGER_PIN 22
#define ECHO_PIN 23

NewPing sonar(TRIGGER_PIN, ECHO_PIN, 200); //max_distance = 200

bool blocked = false;
int target_rotation;  //clockwise direction (+)
int16_t left_ticks_rotation; //left_ticks at start of rotation

// rosserial
//////////////////////////////////////////////////
#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

std_msgs::Int16 left_ticks_msg;
std_msgs::Int16 right_ticks_msg;
ros::Publisher left_ticks("left_ticks", &left_ticks_msg);
ros::Publisher right_ticks("right_ticks", &right_ticks_msg);

//////////////////////////////////////////////////
DCMotor dcMotor1(MOTOR_1A_PIN, MOTOR_1B_PIN, ENABLE_1_PIN);
DCMotor dcMotor2(MOTOR_2A_PIN, MOTOR_2B_PIN, ENABLE_2_PIN);

void doEncoder_1A() {
  dcMotor1.encoder_ticks += (digitalRead(ENCODER_1A_PIN) == digitalRead(ENCODER_1B_PIN)) ? 1 : -1;
}
void doEncoder_1B() {
  dcMotor1.encoder_ticks += (digitalRead(ENCODER_1A_PIN) == digitalRead(ENCODER_1B_PIN)) ? -1 : 1;
}
void doEncoder_2A() {
  dcMotor2.encoder_ticks += (digitalRead(ENCODER_2A_PIN) == digitalRead(ENCODER_2B_PIN)) ? 1 : -1;
}
void doEncoder_2B() {
  dcMotor2.encoder_ticks += (digitalRead(ENCODER_2A_PIN) == digitalRead(ENCODER_2B_PIN)) ? -1 : 1;
}

void setup() {
   Serial.begin(BAUDRATE);

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

  attachInterrupt(digitalPinToInterrupt(ENCODER_1A_PIN), doEncoder_1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_1B_PIN), doEncoder_1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2A_PIN), doEncoder_2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2B_PIN), doEncoder_2B, CHANGE);

  // rosserial
  nh.initNode();
  nh.advertise(left_ticks);
  nh.advertise(right_ticks);
}

void loop() {

  // blocked check
  if (blocked == false) {
    float distance = sonar.ping_cm();
    if (distance < 20) {
      blocked = true;
      // target_rotation = random(-180, 180); //-180 ~ 179
      target_rotation = 90;
      left_ticks_rotation = dcMotor1.encoder_ticks;
    }
  }

  // motor control
  if (blocked == true) {
    // rotating
    if (target_rotation > 0) {
      // turn right
      dcMotor1.control(1, 120);
      dcMotor2.control(1, 120); //reverse direction
    } else {
      // turn left
      dcMotor1.control(2, 120);
      dcMotor2.control(2, 120); //reverse direction
    }
    
    int16_t left_ticks_new = dcMotor1.encoder_ticks - left_ticks_rotation;
    float progressed_rotation = ((left_ticks_new/TICKS_PER_METER) / ((INTERMOTOR_DIST/1000.0)/2))/PI*180;
    Serial.println(progressed_rotation);

    if(progressed_rotation >= target_rotation > 0) {
      blocked = false;
    }
    else if (progressed_rotation <= target_rotation < 0) {
      blocked = false;
    }

  } else {
    // moving forward
    dcMotor1.control(1, 150);
    dcMotor2.control(2, 150); //reverse direction
  }

  // For monitoring in ros
  left_ticks_msg.data = dcMotor1.encoder_ticks;
  right_ticks_msg.data = dcMotor2.encoder_ticks;
  left_ticks.publish(&left_ticks_msg);
  right_ticks.publish(&right_ticks_msg);
  nh.spinOnce();
}