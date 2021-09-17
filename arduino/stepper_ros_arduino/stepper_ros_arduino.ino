#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <Servo.h>
#include <arduino-timer.h>
#include <RotaryEncoder.h>

#define CHANNEL_A_PITCH 2
#define CHANNEL_B_PITCH 3
#define CHANNEL_A_YAW 18
#define CHANNEL_B_YAW 19

#define DIRECTION_PITCH 5
#define PULSE_PITCH 6
#define DIRECTION_YAW 7
#define PULSE_YAW 8

#define PAN 11
#define TILT 12

#define MIN_PULSE_INTERVAL 100
#define DEADZONE 5

RotaryEncoder yaw_encoder(CHANNEL_A_YAW, CHANNEL_B_YAW, RotaryEncoder::LatchMode::TWO03);
RotaryEncoder pitch_encoder(CHANNEL_A_PITCH, CHANNEL_B_PITCH, RotaryEncoder::LatchMode::TWO03);

ros::NodeHandle nh;

struct PitchMotor {
  int pulse_pin, direction_pin;
  int encoder_position = 0, goal_position = 0, calls = 0, pulse_delay = 500;
  bool pulse = false, reached_position = true, pulse_value = HIGH, current_direction = LOW;
  
  PitchMotor(int pulse_pin, int direction_pin) {
    this->pulse_pin = pulse_pin;
    this->direction_pin = direction_pin; 
    
//    pinMode(pulse_pin, OUTPUT);
//    pinMode(direction_pin, OUTPUT);
  }

  void setGoalPosition(int goal_position) {
    this->goal_position = goal_position;
    current_direction = goal_position < encoder_position;
    reached_position = false; 
  }
  
  void checkPosition() {
    pitch_encoder.tick(); 
    encoder_position = pitch_encoder.getPosition();

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
  
};

struct YawMotor {
  int pulse_pin, direction_pin;
  int encoder_position = 0, goal_position = 0, calls = 0, pulse_delay = 500;
  bool pulse = false, reached_position = true, pulse_value = HIGH, current_direction = LOW;
  
  YawMotor(int pulse_pin, int direction_pin) {
    this->pulse_pin = pulse_pin;
    this->direction_pin = direction_pin;

//    pinMode(pulse_pin, OUTPUT);
//    pinMode(direction_pin, OUTPUT);
  }

  void setGoalPosition(int goal_position) {
    this->goal_position = goal_position;
    current_direction = goal_position < encoder_position;
    reached_position = false; 
  }
  
  void checkPosition() {
    yaw_encoder.tick(); 
    encoder_position = yaw_encoder.getPosition();

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
};

YawMotor yaw_motor(PULSE_YAW, DIRECTION_YAW);
PitchMotor pitch_motor(PULSE_PITCH, DIRECTION_PITCH);

Timer<5, micros> timer;

Servo panServo;
Servo tiltServo;

std_msgs::Int32 pitch_msg;
std_msgs::Int32 yaw_msg;

ros::Publisher pitch_encoder_value("right_arm_pitch_position", &pitch_msg);
ros::Publisher yaw_encoder_value("right_arm_yaw_position", &yaw_msg);

void panCallback(const std_msgs::Int16& msg){
  panServo.write(msg.data);  
}

void tiltCallback(const std_msgs::Int16& msg){
  tiltServo.write(msg.data);  
}

void pitchCB(const std_msgs::Int32& msg) {
  int pitchP = -msg.data * (200 * 50/180);
  pitch_motor.setGoalPosition(pitchP);
}

void yawCB(const std_msgs::Int32& msg) {
  int yawP = msg.data * (200 * 50 * 1.65/180);
  yaw_motor.setGoalPosition(yawP);
}

void checkYawPosition() {
  yaw_motor.checkPosition();
}

void checkPitchPosition() {
  pitch_motor.checkPosition();
}

void send_encoder_data() {
  pitch_msg.data = pitch_motor.encoder_position * (180 / (50 * 200));
  pitch_encoder_value.publish(&pitch_msg);

  yaw_msg.data = yaw_motor.encoder_position * (180 / (200 * 50 * 1.65));
  yaw_encoder_value.publish(&yaw_msg);
  return true;
}

bool yaw_motor_call(){
  yaw_motor.motor_call();
  return true;
}

bool pitch_motor_call() {
  pitch_motor.motor_call();
  return true;
}


ros::Subscriber<std_msgs::Int16> pan_sub("pan", &panCallback);
ros::Subscriber<std_msgs::Int16> tilt_sub("tilt", &tiltCallback);

ros::Subscriber<std_msgs::Int32> arm_pitch_position("set_right_arm_pitch_position", &pitchCB);
ros::Subscriber<std_msgs::Int32> arm_yaw_position("set_right_arm_yaw_position", &yawCB);


void setup() {
  pinMode(PULSE_YAW, OUTPUT);
  pinMode(DIRECTION_YAW, OUTPUT);
  pinMode(PULSE_PITCH, OUTPUT);
  pinMode(DIRECTION_PITCH, OUTPUT);

  panServo.attach(PAN);
  tiltServo.attach(TILT);
  
  //setting the pins to output for the motor
  nh.initNode();
  nh.subscribe(arm_pitch_position);
  nh.subscribe(arm_yaw_position);
  nh.subscribe(pan_sub);
  nh.subscribe(tilt_sub);

  nh.advertise(pitch_encoder_value);
  nh.advertise(yaw_encoder_value);

  
  while (!nh.connected()) {
    nh.spinOnce();
    delay(1);
  }
 

  attachInterrupt(digitalPinToInterrupt(CHANNEL_A_PITCH), checkPitchPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHANNEL_B_PITCH), checkPitchPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHANNEL_A_YAW), checkYawPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHANNEL_B_YAW), checkYawPosition, CHANGE);

  timer.every(100, yaw_motor_call);
    timer.every(100, pitch_motor_call);

  timer.every(300000, send_encoder_data);
}

void loop() {
  timer.tick<void>();

  yaw_motor.motor_tick();
  pitch_motor.motor_tick();
  nh.spinOnce();
}
