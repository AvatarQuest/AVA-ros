//#include <ros.h>
//#include <std_msgs/Int32.h>
#include <SoftwareSerial.h>
#include "RoboClaw.h"

#define address 0x80

SoftwareSerial serial(NULL,11);	
SoftwareSerial serial2(NULL, 10);
RoboClaw roboclaw(&serial,10000);
RoboClaw roboclaw2(&serial2,10000);

//int speedm = 0;
//ros::NodeHandle nh;
//
//void drivetrain_speed_callback(const std_msgs::Int32& msg) {
//  speedm = msg.data;
//  if(speedm>=0) roboclaw.ForwardM1(address, speedm);
//  else roboclaw.BackwardM1(address, -speedm);
//}

//ros::Subscriber<std_msgs::Int32> sub("set_speed", &drivetrain_speed_callback);


void setup() {
  //Open roboclaw serial ports
//  nh.initNode();
//  nh.subscribe(sub);
//
//  while (!nh.connected()) {
//    nh.spinOnce();
//    delay(1);
//  }
  roboclaw.begin(38400);
//  roboclaw2.begin(38400);
//  roboclaw.ForwardM1(address, 20);
//  roboclaw2.ForwardM1(address, 20);
  
}

void loop() {
   //start Motor1 forward at half speed
//   roboclaw.ForwardM1(address, 10);
//   nh.spinOnce();
  
  //roboclaw2.ForwardM1(address, 20);
  roboclaw.ForwardM1(address, 50);
//  roboclaw2.ForwardM1(address, 50);

}
