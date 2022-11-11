#include <Servo.h>
#include "ESC.h"
#define LED_PIN (13)
#define SPEED_MIN (1000)
#define SPEED_MAX (2000)   
Servo steer;
ESC drive (3, SPEED_MIN, SPEED_MAX, 500);
void setup() {
  steer.attach(1);
  drive.arm();
}

void loop() {
  drive.speed(SPEED_MAX/2);
  delay(1000);
  for(int i =0; i<5; i++) {
  steer.write(80);
  delay(1000);
  steer.write(120);
  delay(1000);
  }
  steer.detach();
  drive.stop();
  delay(10000000);
}
