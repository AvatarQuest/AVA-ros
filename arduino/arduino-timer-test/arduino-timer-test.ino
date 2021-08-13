#include <arduino-timer.h>
#include <RotaryEncoder.h>

#define CHANNEL_A 2
#define CHANNEL_B 3
#define INDEX 4

#define DIRECTION 5
#define PULSE 6

#define MIN_PULSE_INTERVAL 100
#define DEADZONE 5
Timer<3, micros> timer;

int encoder_position = 0, goal_position = 0, calls = 0, pulse_delay = 400;
bool pulse = false, pulse_value = HIGH, current_direction = LOW;

RotaryEncoder encoder(CHANNEL_A, CHANNEL_B, RotaryEncoder::LatchMode::TWO03);

void checkPosition() {
//  Serial.println("dshfjdsahfjkdsa");
  encoder.tick();
}

bool motor_call(){
  if(calls++ >= pulse_delay/MIN_PULSE_INTERVAL) {
    pulse = true;
    calls = 0;
  }
  return true;
}

bool read_pos() {
  Serial.println( encoder.getPosition() / 50);
  return true;
}

bool change_pd(){
  if(millis() >= 4000) { 
    pulse_delay = 1000;
    current_direction = HIGH;
  }
  return true;
}

void setup() {
  Serial.begin(9600);
  Serial.println("i hope this works! :D");
  //setting the pins to output for the motor
  while (!nodehandle.connected()) {
    nodehandle.spinOnce();
    delay(1);
  }
  
  pinMode(PULSE, OUTPUT);
  pinMode(DIRECTION, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(CHANNEL_A), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHANNEL_B), checkPosition, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(INDEX), rotation, CHANGE);

  
  timer.every(100, motor_call);
  timer.every(100, change_pd);
  timer.every(100000, read_pos);

  

}


void loop() {
   if (!nodehandle.connected()) return;
  timer.tick<void>();
//  encoder.tick();
  digitalWrite(DIRECTION, current_direction);
  if (pulse) {
    digitalWrite(PULSE, pulse_value);
    pulse = false;
    pulse_value = !pulse_value;
//    Serial.println(pulse_value ? "HIGH" : "LOW");  
  }
  nh.spinOnce();
}
