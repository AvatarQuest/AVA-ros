#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>

#define AXIS y

double states[5];

void pinkyCB(const geometry_msgs::Vector3::ConstPtr& msg) {
    
}

void ringCB(const geometry_msgs::Vector3::ConstPtr& msg) {

}

void indexCB(const geometry_msgs::Vector3::ConstPtr& msg) {

}

void middleCB(const geometry_msgs::Vector3::ConstPtr& msg) {

}

void thumbCB(const geometry_msgs::Vector3::ConstPtr& msg) {

}
int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamixel_hand");
    ros::NodeHandle nh;

    ros::Subscriber pinky_sub = nh.subscribe("pinky", 1, pinkyCB);
    ros::Subscriber ring_sub = nh.subscribe("ring", 1, ringCB);
    ros::Subscriber index_sub = nh.subscribe("index", 1,indexCB);
    ros::Subscriber middle_sub = nh.subscribe("middle", 1, middleCB);
    ros::Subscriber thumb_sub = nh.subscribe("thumb", 1, thumbCB);
    ROS_INFO("%s", "starting node 'dynamixel_hand_control'");

    ros::spin();

    return 0;
}