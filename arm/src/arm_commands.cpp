#include <ros/ros.h>
#include <geometry_msgs::Vector3.h>

void positionCB(const geometry_msgs::Vector3::ConstPtr& msg) {

}

ros::Publisher shoulder_pitch_yaw;
ros::Publisher 

void positionCB(const std)

int main(int argc, char** argv) {
    ros::init(arc, argv, "arm_commands");
    ros::NodeHandle nodehandle;
    ros::Subscriber arm_position = nodehandle.subscibe("arm_position_topic", 1, positionCB);
    shoulder_pitch_yaw = nodehandle.advertise<geometry_msgs::Vector3> ("shoulder_pitch_yaw_position", 10);
}