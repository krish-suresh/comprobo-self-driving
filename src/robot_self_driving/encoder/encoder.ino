#include <Encoder.h>

Encoder verticalWheelEncoder(3,2); //TODO: Pins 
Encoder horizontalWheelEncoder(18,19); //TODO: Pins

int hor_enc_power = 13;

long curr_vert_value = 0;
long curr_hor_value = 0;
int prev_time = millis();
int time_threshold = 20;

void setup() {
  // put your setup code here, to run once
  Serial.begin(9600);
  pinMode(hor_enc_power, OUTPUT);
  digitalWrite(hor_enc_power, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  int curr_time = millis();
  if(Serial.availableForWrite()>60 && curr_time - prev_time >= time_threshold)
  {
    curr_vert_value = verticalWheelEncoder.read();
    curr_hor_value = horizontalWheelEncoder.read();
    Serial.println(String(curr_vert_value) + ',' + String(curr_hor_value));
    prev_time = curr_time;
  }
}
