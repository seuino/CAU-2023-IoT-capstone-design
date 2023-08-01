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

// #define BAUDRATE 19200

#include "DCMotor.h"

// rosserial
//////////////////////////////////////////////////
#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle  nh;

std_msgs::Int32 left_ticks_msg;
std_msgs::Int32 right_ticks_msg;
ros::Publisher left_ticks("left_ticks", &left_ticks_msg);
ros::Publisher right_ticks("right_ticks", &right_ticks_msg);

//////////////////////////////////////////////////
DCMotor dcMotor1(MOTOR_1A_PIN, MOTOR_1B_PIN, ENABLE_1_PIN);
DCMotor dcMotor2(MOTOR_2A_PIN, MOTOR_2B_PIN, ENABLE_2_PIN);

void doEncoder_1A() {dcMotor1.encoder_ticks += (digitalRead(ENCODER_1A_PIN)==digitalRead(ENCODER_1B_PIN))?1:-1;}
void doEncoder_1B() {dcMotor1.encoder_ticks += (digitalRead(ENCODER_1A_PIN)==digitalRead(ENCODER_1B_PIN))?-1:1;}
void doEncoder_2A() {dcMotor2.encoder_ticks += (digitalRead(ENCODER_2A_PIN)==digitalRead(ENCODER_2B_PIN))?1:-1;}
void doEncoder_2B() {dcMotor2.encoder_ticks += (digitalRead(ENCODER_2A_PIN)==digitalRead(ENCODER_2B_PIN))?-1:1;}

void setup() {
  // Serial.begin(BAUDRATE);

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
  
  dcMotor1.control(1, 255);
  dcMotor2.control(2, 255);

  // For monitoring in ros
  left_ticks_msg.data = dcMotor1.degree;
  right_ticks_msg.data = dcMotor2.degree;
  left_ticks.publish(&left_ticks_msg);
  right_ticks.publish(&right_ticks_msg);
  nh.spinOnce();

}
