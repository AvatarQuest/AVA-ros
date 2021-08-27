#include <ros.h>
#include <std_msgs/Int32.h>
#include <SoftwareSerial.h>
#include "RoboClaw.h"

#define address 0x80

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(10,11);	
RoboClaw roboclaw(&serial,10000);

int speedm = 0;

ros::NodeHandle nh;

void drivetrain_speed_callback(const std_msgs::Int32& msg) {
  speedm = msg.data;
  if(speedm>=0) roboclaw.ForwardM1(address, speedm);
  else roboclaw.BackwardM1(address, -speedm);
}

ros::Subscriber<std_msgs::Int32> sub("set_speed", &drivetrain_speed_callback);


void setup() {
  //Open roboclaw serial ports
  nh.initNode();
  nh.subscribe(sub);

  while (!nh.connected()) {
    nh.spinOnce();
    delay(1);
  }
  roboclaw.begin(38400);
}

void loop() {
   //start Motor1 forward at half speed
   nh.spinOnce();
}
