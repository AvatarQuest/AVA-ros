/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Int16.h>
#include <Servo.h>

#define pan 5
#define tilt 6

ros::NodeHandle nodehandle;

Servo panServo;
Servo tiltServo;

void panCallback(const std_msgs::Int16& msg){
  panServo.write(msg.data);  
}

void tiltCallback(const std_msgs::Int16& msg){
  tiltServo.write(msg.data);  
}

ros::Subscriber<std_msgs::Int16> pan_sub("pan", &panCallback);
ros::Subscriber<std_msgs::Int16> tilt_sub("tilt", &tiltCallback);

void setup()
{
  Serial.begin(57600);
  panServo.attach(pan);
  tiltServo.attach(tilt);
  
  nodehandle.initNode();
  nodehandle.subscribe(pan_sub);
  nodehandle.subscribe(tilt_sub);
}

void loop()
{
  nodehandle.spinOnce();
  delay(1);
}
