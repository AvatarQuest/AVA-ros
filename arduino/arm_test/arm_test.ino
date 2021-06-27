#include <Arduino.h>
#include <RotaryEncoder.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#define PAN 3
#define TILT 4
#define YAW_PIN_IN1 5 
#define YAW_PIN_IN2 6

#define YAW_PIN 10

#define PITCH_PIN_IN1 7
#define PITCH_PIN_IN2 8

#define PITCH_PIN 11

Servo shoulder_yaw, shoulder_pitch, panServo, tiltServo;
double yaw_goal = 10;
double pitch_goal = 0;

RotaryEncoder yaw_encoder(YAW_PIN_IN1, YAW_PIN_IN2, RotaryEncoder::LatchMode::TWO03);
RotaryEncoder pitch_encoder(PITCH_PIN_IN1, PITCH_PIN_IN2, RotaryEncoder::LatchMode::TWO03);

ros::NodeHandle nodehandle;

void setShoulderYawCallback(const std_msgs::Float64& msg){
    yaw_goal = msg.data;
    
}
void setShoulderPitchCallback(const std_msgs::Float64& msg){
    if(abs(msg.data>210)) {
      pitch_goal=210*msg.data/abs(msg.data); //positive/negative operator
    } else pitch_goal = msg.data;
}

void panCallback(const std_msgs::Int32& msg){
  panServo.write(msg.data);  
}

void tiltCallback(const std_msgs::Int32& msg){
  tiltServo.write(msg.data);  
}

void checkYawPosition() {
  yaw_encoder.tick(); 
}

void checkPitchPosition() {
  pitch_encoder.tick(); 
} 

ros::Subscriber<std_msgs::Float64> setShoulderYaw("shoulder_yaw", &setShoulderYawCallback);
ros::Subscriber<std_msgs::Float64> setShoulderPitch("shoulder_pitch", &setShoulderPitchCallback);

ros::Subscriber<std_msgs::Int32> pan_sub("pan", &panCallback);
ros::Subscriber<std_msgs::Int32> tilt_sub("tilt", &tiltCallback);

void setup()
{
  Serial.begin(57600);
  shoulder_yaw.attach(YAW_PIN);
  shoulder_pitch.attach(PITCH_PIN);
  panServo.attach(PAN);
  tiltServo.attach(TILT);

  nodehandle.initNode();
  
  nodehandle.subscribe(setShoulderYaw);
  nodehandle.subscribe(setShoulderPitch);

  nodehandle.subscribe(pan_sub);
  nodehandle.subscribe(tilt_sub);

  while (!nodehandle.connected()) {
    nodehandle.spinOnce();
    delay(1);
  }
  
//  attachInterrupt(digitalPinToInterrupt(YAW_PIN_IN1), checkYawPosition, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(YAW_PIN_IN2), checkYawPosition, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(PITCH_PIN_IN1), checkPitchPosition, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(PITCH_PIN_IN2), checkPitchPosition, CHANGE);
}


void loop(){
  int yaw = yaw_encoder.getPosition();
  int pitch = pitch_encoder.getPosition();
  int yaw_speed = yaw < yaw_goal ? 96 : 87; 
  int pitch_speed = pitch < pitch_goal ? 97 : 88; 

  if (abs(yaw - yaw_goal) < 7) { yaw_speed = 90; }
  if (abs(pitch - pitch_goal) < 7) { pitch_speed = 90; }

  shoulder_yaw.write(yaw_speed);
  shoulder_pitch.write(pitch_speed);
  nodehandle.spinOnce();
  pitch_encoder.tick(); 
  yaw_encoder.tick(); 
  delay(1);
}
