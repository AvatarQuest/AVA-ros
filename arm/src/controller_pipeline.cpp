#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include <sstream>


ros::Publisher shoulder_pitch_command;
ros::Publisher shoulder_yaw_command;
ros::Publisher elbow_command;
ros::Publisher wrist_command;

void pipelineCB(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    std_msgs::Float64 shoulder_pitch_msg;
    std_msgs::Float64 shoulder_yaw_msg;
    std_msgs::Float64 elbow_msg;
    std_msgs::Float64 wrist_msg;

    for (auto d : msg->data) {
        ROS_INFO("INCOMING DATA %.2f", d+2);
    }

    shoulder_pitch_msg.data = msg->data[0] + 2;
    shoulder_yaw_msg.data = msg->data[1] + 2;
    elbow_msg.data = msg->data[2] + 2;
    wrist_msg.data = msg->data[3] + 2;

    shoulder_pitch_command.publish(shoulder_pitch_msg);
    shoulder_yaw_command.publish(shoulder_yaw_msg);
    elbow_command.publish(elbow_msg);
    wrist_command.publish(wrist_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller_pipeline");
    ros::NodeHandle nodehandle;

    ros::Subscriber pipeline = nodehandle.subscribe("controller_pipeline/command", 10, pipelineCB);
    shoulder_pitch_command = nodehandle.advertise<std_msgs::Float64>("shoulder_pitch_controller/command", 1);
    shoulder_yaw_command = nodehandle.advertise<std_msgs::Float64>("shoulder_yaw_controller/command", 1);
    elbow_command = nodehandle.advertise<std_msgs::Float64>("elbow_controller/command", 1);
    wrist_command = nodehandle.advertise<std_msgs::Float64>("wrist_controller/command", 1);
    
    ros::spin();
    return 0;
}