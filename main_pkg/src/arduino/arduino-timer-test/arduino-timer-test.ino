#include <ros.h>
#include <std_msgs/Int32.h>
#include <arduino-timer.h>
#include <RotaryEncoder.h>

#define CHANNEL_A 2
#define CHANNEL_B 3
#define INDEX 4

#define DIRECTION 5
#define PULSE 6

#define MIN_PULSE_INTERVAL 100
#define DEADZONE 5
Timer<4, micros> timer;

int encoder_position = 0, goal_position = 0, calls = 0, pulse_delay = 500;
bool pulse = false, reached_position = false, pulse_value = HIGH, current_direction = LOW;

RotaryEncoder encoder(CHANNEL_A, CHANNEL_B, RotaryEncoder::LatchMode::TWO03);
ros::NodeHandle nh;
std_msgs::Int32 int_msg;

ros::Publisher encoder_value("right_arm_pitch_position", &int_msg);

void callback(const std_msgs::Int32& msg) {
  goal_position = msg.data;
  current_direction = goal_position < encoder_position;
  reached_position = false; 
}

void checkPosition() {
  encoder.tick(); 
  encoder_position = encoder.getPosition();
  
  if (abs(goal_position - encoder_position) < DEADZONE) {
    reached_position = true; 
  }

  return true;
}

void send_encoder_data() {
  int_msg.data = encoder_position;
  encoder_value.publish(&int_msg);
  return true;
}

bool motor_call(){
  if(calls++ >= pulse_delay/MIN_PULSE_INTERVAL) {
    pulse = true;
    calls = 0;
  }
  return true;
}

ros::Subscriber<std_msgs::Int32> arm_pitch_position("set_right_arm_pitch_position", &callback);

void setup() {
  //setting the pins to output for the motor
  nh.initNode();
  nh.subscribe(arm_pitch_position);
  nh.advertise(encoder_value);
  
  while (!nh.connected()) {
    nh.spinOnce();
    delay(1);
  }
  
  pinMode(PULSE, OUTPUT);
  pinMode(DIRECTION, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(CHANNEL_A), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHANNEL_B), checkPosition, CHANGE);

  timer.every(100, motor_call);
  timer.every(200000, send_encoder_data);
}

void loop() {
  timer.tick<void>();

  digitalWrite(DIRECTION, current_direction);
  if (!reached_position) {
    if (pulse) {
      digitalWrite(PULSE, pulse_value);
      pulse = false;
      pulse_value = !pulse_value;  
    }
  }
  nh.spinOnce();
}
