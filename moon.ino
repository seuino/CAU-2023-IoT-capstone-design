#define USE_USBCON

// dcmotor
#include "DCMotor.h"
// servo
#include <Servo.h>
// rosserial
#include <Wire.h>
//i2c
//#include <Adafruit_MLX90614.h>
//mlx90614
#include <DHT.h>
//dht11
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h> 

//////////////////////////////////////////////////
//#define SERIAL_DEBUG
#define BAUDRATE 57600

#ifndef SERIAL_DEBUG
  #define USE_USBCON
#endif

#define INTERVAL 10 //ms

// dcmotor
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

// const float PI = 3.141592;

const float drift_multiplier = 10;

const uint8_t CPR = 12;
const float GEAR_RATIO = 210.59;
const float WHEEL_DIA = 0.060; //m
const float INTERMOTOR_DIST = 0.210; //m

const float TICKS_PER_RADIAN = GEAR_RATIO*CPR/PI;
const float TICKS_PER_METER = GEAR_RATIO*CPR/WHEEL_DIA/PI; //(TICKS_PER_RADIAN*PI)*(1/(WHEEL_DIA*PI))

const float LEFT_K_B1 = 0;
const float LEFT_K_B2 = 0;
const float RIGHT_K_B1 = 0;
const float RIGHT_K_B2 = 0;
const float PUMP1_K_B1 = 0;
const float PUMP1_K_B2 = 0;
const float PUMP2_K_B1 = 0;
const float PUMP2_K_B2 = 0;

// servo
#define SERVO_1_PIN 6
#define SERVO_2_PIN 7

//////////////////////////////////////////////////
// dcmotor
DCMotor left_dcmotor(MOTOR_1A_PIN, MOTOR_1B_PIN, ENABLE_1_PIN, LEFT_K_B1, LEFT_K_B2);
DCMotor right_dcmotor(MOTOR_2A_PIN, MOTOR_2B_PIN, ENABLE_2_PIN, RIGHT_K_B1, RIGHT_K_B2);

DCMotor pump1(MOTOR_3A_PIN, MOTOR_3B_PIN, ENABLE_3_PIN, PUMP1_K_B1, PUMP1_K_B2);
DCMotor pump2(MOTOR_4A_PIN, MOTOR_4B_PIN, ENABLE_4_PIN, PUMP2_K_B1, PUMP2_K_B2);

void doEncoder_1A() {left_dcmotor.encoder_ticks += (digitalRead(ENCODER_1A_PIN)==digitalRead(ENCODER_1B_PIN))?1:-1;}
void doEncoder_1B() {left_dcmotor.encoder_ticks += (digitalRead(ENCODER_1A_PIN)==digitalRead(ENCODER_1B_PIN))?-1:1;}
void doEncoder_2A() {right_dcmotor.encoder_ticks += (digitalRead(ENCODER_2A_PIN)==digitalRead(ENCODER_2B_PIN))?1:-1;}
void doEncoder_2B() {right_dcmotor.encoder_ticks += (digitalRead(ENCODER_2A_PIN)==digitalRead(ENCODER_2B_PIN))?-1:1;}

// servo
Servo servo1;
Servo servo2;

// rosserial
ros::NodeHandle  nh;

std_msgs::Int16 left_ticks_msg;
std_msgs::Int16 right_ticks_msg;
ros::Publisher left_ticks_pub("left_ticks", &left_ticks_msg);
ros::Publisher right_ticks_pub("right_ticks", &right_ticks_msg);


float linear_x = 0;
float angular_z = 0;

void getCmdVel(const geometry_msgs::Twist& data) {
  linear_x = data.linear.x;
  angular_z = data.angular.z;
}


ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &getCmdVel);

uint8_t pump1_pwm = 0;
uint8_t pump2_pwm = 0;
uint8_t servo1_pwm = 90;
uint8_t servo2_pwm = 90;

void getPump1Pwm(const std_msgs::UInt8& data) {
  pump1_pwm = data.data;
}
void getPump2Pwm(const std_msgs::UInt8& data) {
  pump2_pwm = data.data;
}
void getServo1Pwm(const std_msgs::UInt8& data) {
  servo1_pwm = data.data;
}
void getServo2Pwm(const std_msgs::UInt8& data) {
  servo2_pwm = data.data;
}
ros::Subscriber<std_msgs::UInt8> pump1_pwm_sub("pump1_pwm", &getPump1Pwm);
ros::Subscriber<std_msgs::UInt8> pump2_pwm_sub("pump2_pwm", &getPump2Pwm);
ros::Subscriber<std_msgs::UInt8> servo1_pwm_sub("servo1_pwm", &getServo1Pwm);
ros::Subscriber<std_msgs::UInt8> servo2_pwm_sub("servo2_pwm", &getServo2Pwm);



/////// temperature&humidity///////////////////////////////////////

//Adafruit_MLX90614 mlx = Adafruit_MLX90614();
#define DHTPIN A0
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);


std_msgs::Float32 temperature_msg;
std_msgs::Float32 humidity_msg;
std_msgs::String emergency_msg; 

ros::Publisher temperature_pub("temperature", &temperature_msg);
ros::Publisher humidity_pub("humidity", &humidity_msg);
ros::Publisher emergency_pub("emergency", &emergency_msg); 

const float threshold_temperature = 29.0; 
//////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////
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
  
  
  ////////////tem&hum///////////////
  //mlx.begin();
  dht.begin();
  ////////////////////////////////////

  // rosserial
  nh.getHardware()->setBaud(BAUDRATE);
  nh.initNode();
  nh.advertise(left_ticks_pub);
  nh.advertise(right_ticks_pub);
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(pump1_pwm_sub);
  nh.subscribe(pump2_pwm_sub);
  nh.subscribe(servo1_pwm_sub);
  nh.subscribe(servo2_pwm_sub);

  
  nh.advertise(temperature_pub);
  nh.advertise(humidity_pub);
  nh.advertise(emergency_pub); 

}

void loop() {
  nh.spinOnce();



  uint32_t current_millis = millis();
  static uint32_t previous_millis = current_millis;
  if(current_millis - previous_millis > INTERVAL) {
    previous_millis = current_millis;

    //////////////////////////////////////////////////
    //                   Control                    //
    //////////////////////////////////////////////////
    // left_dcmotor.control_pwm(1, 255);
    // right_dcmotor.control_pwm(1, 255);
    achieveCmdVel("pid");
    // pump1.control_pwm(1, 255);
    // pump2.control_pwm(1, 255);
    // servo1.write(90);
    // servo2.write(90);
    achieveCleanerAction();
    
    //////////////////////////////////////////////////
    //                     ROS                      //
    //////////////////////////////////////////////////
    left_ticks_msg.data = left_dcmotor.encoder_ticks;
    right_ticks_msg.data = right_dcmotor.encoder_ticks;
    left_ticks_pub.publish(&left_ticks_msg);
    right_ticks_pub.publish(&right_ticks_msg);
    
    #ifdef SERIAL_DEBUG
      Serial.print("left: ");
      Serial.print(left_dcmotor.encoder_ticks);
      Serial.print(",");
      Serial.print(left_dcmotor.velocity);
      Serial.print("\tright: ");
      Serial.print(right_dcmotor.encoder_ticks);
      Serial.print(",");
      Serial.print(right_dcmotor.velocity);
      Serial.println();
    #endif


      ///////////////////////tem&hum////////////////////////////
       // Measure temperature using DHT11
       float temperature = dht.readTemperature(); 
       temperature_msg.data = temperature;
       temperature_pub.publish(&temperature_msg);

       // Measure humidity using DHT11
       float humidity = dht.readHumidity();
       humidity_msg.data = humidity;
       humidity_pub.publish(&humidity_msg);

       // Check if temperature exceeds threshold
       if (temperature > threshold_temperature) {
         emergency_msg.data = "emergency"; 
          emergency_pub.publish(&emergency_msg);
       }
      /////////////////////////////////////////////////////////////
  }

}

// Follow linear_x, angular_z
// "pid" for PID control
// "linear" for linear control
void achieveCmdVel(const char *control_type) {
  float left_target_velocity = linear_x - (angular_z * (INTERMOTOR_DIST/2));
  float right_target_velocity = linear_x + (angular_z * (INTERMOTOR_DIST/2));

  if(control_type == "pid") {
    left_dcmotor.calcVel();
    right_dcmotor.calcVel();

    int16_t left_pwm = left_dcmotor.calcPid(left_target_velocity);
    int16_t right_pwm = right_dcmotor.calcPid(right_target_velocity);

    left_dcmotor.controlPwm(left_pwm);
    right_dcmotor.controlPwm(right_pwm);

    #ifdef SERIAL_DEBUG  
      Serial.print("pwm: ");
      Serial.print(left_pwm);
      Serial.print(",");
      Serial.print(right_pwm);
      Serial.print("\t");
    #endif
  }
  else if(control_type == "linear") {
    left_dcmotor.calcVel();
    right_dcmotor.calcVel();

    int16_t left_pwm = left_dcmotor.calcLinear(left_target_velocity);
    int16_t right_pwm = right_dcmotor.calcLinear(right_target_velocity);

    if(angular_z == 0) { //only moving forward
      // Remove any differences in wheel velocities 
      // to make sure the robot goes straight
      static float pre_velocity_diff = 0;
      static float pre_pre_velocity_diff = 0;
      float velocity_diff = left_dcmotor.velocity - right_dcmotor.velocity; 
      double velocity_diff_avg = (velocity_diff+pre_velocity_diff+pre_pre_velocity_diff)/3;

      pre_pre_velocity_diff = pre_velocity_diff;
      pre_velocity_diff = velocity_diff;
  
      left_pwm -= (int16_t)(velocity_diff_avg * drift_multiplier);
      right_pwm += (int16_t)(velocity_diff_avg * drift_multiplier);

      #ifdef SERIAL_DEBUG  
        Serial.print("pwm: ");
        Serial.print(left_pwm);
        Serial.print(",");
        Serial.print(right_pwm);
        Serial.print("\t");
      #endif
    }

    // If the required PWM is of opposite sign as the output PWM, we want to
    // stop the car before switching direction
    static int pre_left_pwm = 0;
    static int pre_right_pwm = 0;
    
    if ((left_pwm * left_dcmotor.velocity < 0 && pre_left_pwm != 0) ||
        (right_pwm * right_dcmotor.velocity < 0 && pre_right_pwm != 0)) {
      left_pwm = 0;
      right_pwm = 0;
    }

    left_dcmotor.controlPwm(left_pwm);
    right_dcmotor.controlPwm(right_pwm);
  }
}

void achieveCleanerAction() {
  pump1.controlPwm(pump1_pwm);
  pump2.controlPwm(pump2_pwm);
  servo1.write(servo1_pwm);
  servo2.write(servo2_pwm);
}
