#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <SoftwareSerial.h>
#include "RoboClaw.h"

#define address 0x80

//left
SoftwareSerial serial1(NULL,10);	
SoftwareSerial serial2(NULL,11);  
//right
SoftwareSerial serial3(NULL,12);  
SoftwareSerial serial4(NULL,13);  

//left
RoboClaw roboclaw1(&serial1,10000);
RoboClaw roboclaw2(&serial2,10000);
//right
RoboClaw roboclaw3(&serial3,10000);
RoboClaw roboclaw4(&serial4,10000);

int leftSpeed = 0;
int rightSpeed = 0;

ros::NodeHandle nh;

void drivetrain_speed_callback(const geometry_msgs::Vector3& msg) {
  leftSpeed = msg.x;
  rightSpeed = msg.y;

  if(leftSpeed>=0) { roboclaw1.ForwardM1(address, leftSpeed); roboclaw2.ForwardM1(address, leftSpeed); }
  else { roboclaw1.BackwardM1(address, -leftSpeed); roboclaw2.BackwardM1(address, -leftSpeed); }

  if(rightSpeed>=0) { roboclaw3.ForwardM1(address, rightSpeed); roboclaw4.ForwardM1(address, rightSpeed); }
  else { roboclaw3.BackwardM1(address, -rightSpeed); roboclaw4.BackwardM1(address, -rightSpeed); }
}

ros::Subscriber<geometry_msgs::Vector3> sub("set_drivetrain_speed", &drivetrain_speed_callback);


void setup() {
  //Open roboclaw serial ports
  nh.initNode();
  nh.subscribe(sub);

  while (!nh.connected()) {
    nh.spinOnce();
    delay(1);
  }
  roboclaw1.begin(38400);
  roboclaw2.begin(38400);
  roboclaw3.begin(38400);
  roboclaw4.begin(38400);
}

void loop() {
   //start Motor1 forward at half speed
   nh.spinOnce();
}
