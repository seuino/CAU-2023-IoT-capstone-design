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

#define GEAR_RATIO 986.61
#define CPR 12

// #define BAUDRATE 19200

// rosserial
//////////////////////////////////////////////////
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char buffer[10];
//////////////////////////////////////////////////

// .h
//////////////////////////////////////////////////
class DCMotor {
  public:
    DCMotor(uint8_t motorA_pin, uint8_t motorB_pin, uint8_t enable_pin,
            uint8_t motor_num) {

      _motorA_pin = motorA_pin;
      _motorB_pin = motorB_pin;
      _enable_pin = enable_pin;
      
      _motor_num = motor_num;
    }

    void control(int key, uint8_t pwm);

    volatile int32_t encoder_pos = 0;
    float degree = 0;

  private:
    uint8_t _motorA_pin;
    uint8_t _motorB_pin;
    uint8_t _enable_pin;

    uint8_t _motor_num;    
};
// .cpp
//////////////////////////////////////////////////
void DCMotor::control(int key, uint8_t pwm) {

  degree = float(encoder_pos) * 360/GEAR_RATIO/CPR;

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

  // For monitoring in ros
  String str = String(degree, 2);
  str.toCharArray(buffer, 10);
  str_msg.data = buffer;
  chatter.publish( &str_msg );
  nh.spinOnce();

}
//////////////////////////////////////////////////
DCMotor dcMotor1(MOTOR_1A_PIN, MOTOR_1B_PIN, ENABLE_1_PIN, 1);
DCMotor dcMotor2(MOTOR_2A_PIN, MOTOR_2B_PIN, ENABLE_2_PIN, 2);

void doEncoder_1A() {dcMotor1.encoder_pos += (digitalRead(ENCODER_1A_PIN)==digitalRead(ENCODER_1B_PIN))?1:-1;}
void doEncoder_1B() {dcMotor1.encoder_pos += (digitalRead(ENCODER_1A_PIN)==digitalRead(ENCODER_1B_PIN))?-1:1;}
void doEncoder_2A() {dcMotor2.encoder_pos += (digitalRead(ENCODER_2A_PIN)==digitalRead(ENCODER_2B_PIN))?1:-1;}
void doEncoder_2B() {dcMotor2.encoder_pos += (digitalRead(ENCODER_2A_PIN)==digitalRead(ENCODER_2B_PIN))?-1:1;}

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
  nh.advertise(chatter);
}

void loop() {
  
  dcMotor1.control(1, 255);
  dcMotor2.control(2, 255);

}
