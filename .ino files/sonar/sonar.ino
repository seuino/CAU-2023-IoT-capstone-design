// serial
//////////////////////////////////////////////////
#define BAUDRATE 19200
// #define USE_USBCON

// sonar sensor
//////////////////////////////////////////////////
#include <NewPing.h>

#define TRIGGER_PIN 22
#define ECHO_PIN 23

NewPing sonar(TRIGGER_PIN, ECHO_PIN, 200); //max_distance = 200

void setup() {
  Serial.begin(BAUDRATE);
}

void loop() {

  float distance = sonar.ping_cm();
  Serial.println(distance);

}