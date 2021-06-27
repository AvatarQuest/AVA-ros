#include <vector>
#include "ros/ros.h"
#include "drivetrain/Motor.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"
#include <signal.h>

#include <sstream>

typedef struct Wheel {
  int motor1;
  int motor2;

  Wheel(int motor1, int motor2) {
    this->motor1 = motor1;
    this->motor2 = motor2;
  };
} Wheel;

ros::Publisher motor_speed_topic;

Wheel wheel0 = Wheel(3, 5);
Wheel wheel1 = Wheel(6, 7);
Wheel wheel2 = Wheel(8, 9);
Wheel wheel3 = Wheel(10, 11);

const std::vector<Wheel> gearbox_ids = {wheel0, wheel1, wheel2, wheel3};
const std::vector<int> drivetrain_ids = {3, 5, 6, 7, 8, 9, 10, 11};

void wheelCallback(const drivetrain::Motor::ConstPtr& msg) {
  drivetrain::Motor motor1;
  drivetrain::Motor motor2;
  if (msg->id > 4 && msg->id <= 0){
    return;
  }
  motor1.id = gearbox_ids[msg->id].motor1;  
  motor2.id = gearbox_ids[msg->id].motor2;
  ROS_INFO("Motor 1: %d Motor 2: %d, speed: %d", motor1.id, motor2.id, msg->speed);

  motor1.speed = msg->speed;
  motor2.speed = msg->speed;

  motor_speed_topic.publish(motor1);
  motor_speed_topic.publish(motor2);
}

void rightSideCallback(const std_msgs::Float64::ConstPtr& msg) {
  drivetrain::Motor motor1, motor2, motor3, motor4;
  double speed = msg->data * 65;

  motor1.id = gearbox_ids[0].motor1;
  motor2.id = gearbox_ids[0].motor2;
  motor3.id = gearbox_ids[1].motor1;
  motor4.id = gearbox_ids[1].motor2;
  
  motor1.speed = speed;
  motor2.speed = speed;
  motor3.speed = speed;
  motor4.speed = speed;

  motor_speed_topic.publish(motor1);
  motor_speed_topic.publish(motor2);
  motor_speed_topic.publish(motor3);
  motor_speed_topic.publish(motor4);

  ROS_INFO("Right Speed: %d", msg->data);
}

void leftSideCallback(const std_msgs::Float64::ConstPtr& msg) {
  drivetrain::Motor motor1, motor2, motor3, motor4;
  double speed = msg->data * 65;

  motor1.id = gearbox_ids[2].motor1;
  motor2.id = gearbox_ids[2].motor2;
  motor3.id = gearbox_ids[3].motor1;
  motor4.id = gearbox_ids[3].motor2;
  
  motor1.speed = speed;
  motor2.speed = speed;
  motor3.speed = speed;
  motor4.speed = speed;

  motor_speed_topic.publish(motor1);
  motor_speed_topic.publish(motor2);
  motor_speed_topic.publish(motor3);
  motor_speed_topic.publish(motor4);

  ROS_INFO("Left Speed: %d", msg->data);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "wheel_control");
    ros::NodeHandle nodehandle;
    
    // nodehandle.setParam("drivetrain/gearbox_ids", gearbox_ids);

    //ros::Subscriber speed_topic = nodehandle.subscribe("set_wheel_speed", 1, wheelCallback);
    ros::Subscriber left_side = nodehandle.subscribe("left_side_speed", 1, leftSideCallback);
    ros::Subscriber right_side = nodehandle.subscribe("right_side_speed", 1, rightSideCallback);
    motor_speed_topic = nodehandle.advertise<drivetrain::Motor>("set_motor_speed", 1);
    
    std::cout << "Started node: 'wheel control'" << std::endl;

    ros::spin();
    return 0;
}