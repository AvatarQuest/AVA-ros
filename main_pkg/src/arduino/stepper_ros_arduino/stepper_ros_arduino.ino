
#include <ros.h>
#include <std_msgs/Int32.h>
#include <arduino-timer.h>
#include <RotaryEncoder.h>

#define CHANNEL_A_PITCH 2
#define CHANNEL_B_PITCH 3
#define INDEX_PITCH 4

#define DIRECTION_PITCH 5
#define PULSE_PITCH 6

#define CHANNEL_A_YAW 18
#define CHANNEL_B_YAW 19
#define INDEX_YAW 4

#define DIRECTION_YAW 5
#define PULSE_YAW 6

#define MIN_PULSE_INTERVAL 100
#define DEADZONE 5

struct pitchMotor {
  int pulse_pin, direction_pin, encoder_a, encoder_b;
  RotaryEncoder encoder;
  int encoder_position = 0, goal_position = 0, calls = 0, pulse_delay = 500;
  bool pulse = false, reached_position = false, pulse_value = HIGH, current_direction = LOW;
  
  pitchMotor(int pulse_pin, int direction_pin) {
    this->pulse_pin = pulse_pin;
    this->direction_pin = direction_pin;
    this->encoder_a = encoder_a;
    this->encoder_b = encoder_b;    
    
    pinMode(pulse_pin, OUTPUT);
    pinMode(direction_pin, OUTPUT);
  }

  void setGoalPosition(int goal_position) {
    current_direction = goal_position < encoder_position;
    reached_position = false; 
  }
  void checkPosition() {
    encoder.tick(); 
    encoder_position = encoder.getPosition();

    if (abs(goal_position - encoder_position) < DEADZONE) {
      reached_position = true; 
    }
  }

  void motor_tick() {
    digitalWrite(direction_pin, current_direction);
    if (!reached_position) {
      if (pulse) {
        digitalWrite(pulse_pin, pulse_value);
        pulse = false;
        pulse_value = !pulse_value;  
      }
    }
  }

  void motor_call() {
    if(calls++ >= pulse_delay/MIN_PULSE_INTERVAL) {
      pulse = true;
      calls = 0;
    }
  }
  
}pitch;

struct yawMotor {
  int pulse_pin, direction_pin, encoder_a, encoder_b;
  RotaryEncoder encoder;
  int encoder_position = 0, goal_position = 0, calls = 0, pulse_delay = 500;
  bool pulse = false, reached_position = false, pulse_value = HIGH, current_direction = LOW;
  
  yawMotor(int pulse_pin, int direction_pin) {
    this->pulse_pin = pulse_pin;
    this->direction_pin = direction_pin;
    this->encoder_a = encoder_a;
    this->encoder_b = encoder_b;    
    this->encoder = encoder;

    pinMode(pulse_pin, OUTPUT);
    pinMode(direction_pin, OUTPUT);
  }

  void setGoalPosition(int goal_position) {
    current_direction = goal_position < encoder_position;
    reached_position = false; 
  }
  void checkPosition() {
    encoder.tick(); 
    encoder_position = encoder.getPosition();

    if (abs(goal_position - encoder_position) < DEADZONE) {
      reached_position = true; 
    }
  }

  void motor_tick() {
    digitalWrite(direction_pin, current_direction);
    if (!reached_position) {
      if (pulse) {
        digitalWrite(pulse_pin, pulse_value);
        pulse = false;
        pulse_value = !pulse_value;  
      }
    }
  }

  void motor_call() {
    if(calls++ >= pulse_delay/MIN_PULSE_INTERVAL) {
      pulse = true;
      calls = 0;
    }
  }
  
}yaw;

Timer<4, micros> timer;

//int encoder_position = 0, goal_position = 0, calls = 0, pulse_delay = 500;
//bool pulse = false, reached_position = false, pulse_value = HIGH, current_direction = LOW;

ros::NodeHandle nh;
std_msgs::Int32 int_msg;

ros::Publisher encoder_value("right_arm_pitch_position", &int_msg);

void callback(const std_msgs::Int32& msg) {
//  goal_position = msg.data;
//  current_direction = goal_position < encoder_position;
//  reached_position = false; 
}

void checkPosition() {
//  encoder.tick(); 
//  encoder_position = encoder.getPosition();
//  
//  if (abs(goal_position - encoder_position) < DEADZONE) {
//    reached_position = true; 
//  }
//
//  return true;
}

void send_encoder_data() {
//  int_msg.data = encoder_position;
//  encoder_value.publish(&int_msg);
//  return true;
}

bool motor_call(){
//  if(calls++ >= pulse_delay/MIN_PULSE_INTERVAL) {
//    pulse = true;
//    calls = 0;
//  }
//  return true;
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
 

//  attachInterrupt(digitalPinToInterrupt(CHANNEL_A_PITCH), checkPosition, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(CHANNEL_B_PITCH), checkPosition, CHANGE);

  timer.every(100, motor_call);
  timer.every(200000, send_encoder_data);
}

void loop() {
  timer.tick<void>();

  yaw.motor_tick();
  pitch.motor_tick();
  nh.spinOnce();
}
