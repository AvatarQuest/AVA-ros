#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "DynamixelHelper.hpp"

#include <iostream>
#include <sstream>

MotorIdentifier elbow1;
MotorIdentifier elbow2;
MotorIdentifier wrist;
MotorIdentifier claw;

DynamixelHelper helper;

void elbowCallback(const std_msgs::Float64::ConstPtr& msg) {
    double angle1 = msg->data; // for motor 1
    double angle2 = 133.8 - (angle1 - 180); // for motor 2 (offset and reversed)

    helper.setAngle(elbow1, angle1);
    helper.setAngle(elbow2, angle2);

    ROS_INFO("Setting elbow angle: %.2f (Elbow 1: %.2f, Elbow 2: %.2f)", angle1, angle1, angle2);
}

void wristCallback(const std_msgs::Float64::ConstPtr& msg) {
    double angle = msg->data;
    helper.setAngle(wrist, angle);
    ROS_INFO("Setting wrist angle: %.2f", angle);
}

void clawCallback(const std_msgs::Float64::ConstPtr& msg) {
    double angle = msg->data;
    helper.setAngle(claw, angle);
    ROS_INFO("Setting claw angle: %.2f", angle);
}

int main(int argc, char **argv) {
    const std::string port = "/dev/ttyUSB0";
    const uint baudrate = 57600;
    const AddressTableBase table = XM430W350T_TABLE();

    std::unordered_map<MotorIdentifier, AddressTableBase, MotorIndentifierHasher> motors;

    elbow1 = MotorIdentifier(12, baudrate, port);
    elbow2 =  MotorIdentifier(13, baudrate, port);
    wrist =  MotorIdentifier(14, baudrate, port);
    claw =  MotorIdentifier(15, baudrate, port);

    motors[elbow1] = table;
    motors[elbow2] = table;
    motors[wrist] = table;
    motors[claw] = table;

    helper = DynamixelHelper(motors);

    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle nodehandle;

    ros::Subscriber elbow_angle = nodehandle.subscribe("elbow_angle", 20, elbowCallback);
    ros::Subscriber wrist_angle = nodehandle.subscribe("wrist_angle", 20, wristCallback);
    ros::Subscriber claw_topic = nodehandle.subscribe("claw_position", 20, clawCallback);

    helper.setTorque(elbow1, true);
    helper.setTorque(elbow2, true);
    // helper.setAngle(elbow2, 133.8);
    // helper.setTorque(wrist, true);
    // helper.setTorque(claw, true);
    ROS_INFO("started node 'dynamixel controller'");
    ros::spin();
}