#include <Encoder.h>

// Instantiate Encoder objects based on connected pins
Encoder verticalWheelEncoder(3,2);  
Encoder horizontalWheelEncoder(18,19); 

// Due to a lack of power pins on the arduino, one encoder
// is powered by a GPIO pin set to high
int hor_enc_power = 13;

// Declare variables to store each encoder's current ticks
long curr_vert_value = 0;
long curr_hor_value = 0;

// Set timer variables to ensure that serial isn't sent too
// frequently
int prev_time = millis();
int time_threshold = 20;

void setup() {
  // Setup serial
  Serial.begin(9600);

  // Set horizontal encoder power to HIGH
  pinMode(hor_enc_power, OUTPUT);
  digitalWrite(hor_enc_power, HIGH);
}

void loop() {
  // If enough time has passed, read the current value of each encoder and send it
  // via serial
  int curr_time = millis();
  if(Serial.availableForWrite()>60 && curr_time - prev_time >= time_threshold)
  {
    curr_vert_value = verticalWheelEncoder.read();
    curr_hor_value = horizontalWheelEncoder.read();
    Serial.println(String(curr_vert_value) + ',' + String(curr_hor_value));
    prev_time = curr_time;
  }
}
