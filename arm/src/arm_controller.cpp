#include <iostream>
#include <math.h>

#include <arm_controller/IKServer.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

/*
fix the offsets and the axis through testing

*/
//constants for the length
double a1 = 19.5, a2 = 23.7, a3 = 0.01;
ros::Publisher shoulderPitch, shoulderYaw, elbow;
ik3d::IKServer server;
int yawState = 0, pitchState = 0;
void ik3d::IKServer::setRotationAngle(double angle) {
    if (yawState != int(angle)) {
    	std_msgs::Int32 msg;
    	msg.data = angle;
    	shoulderYaw.publish(msg);
	yawState = angle;
    }
}

void ik3d::IKServer::setElbowAngle(double angle) {
    std_msgs::Float64 msg;
    msg.data = angle;
    elbow.publish(msg);
}

void ik3d::IKServer::setShoulderAngle(double angle) {
    if (pitchState != int(angle)) {
    	std_msgs::Int32 msg;
    	msg.data = angle;
    	shoulderPitch.publish(msg);
	pitchState = angle;
    }
}


void IKCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    double x_component = msg->x;
    double y_component = msg->y;
    double z_component = msg->z;
    server.moveArm(x_component, y_component, z_component);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle nh;
    ros::Subscriber arm_control = nh.subscribe("arm_position",1,IKCallback);

    server = ik3d::IKServer(a1, a2, a3);
    server.setDebug(true);

    shoulderYaw =  nh.advertise<std_msgs::Int32>("set_right_arm_yaw_position",1);
    shoulderPitch = nh.advertise<std_msgs::Int32>("set_right_arm_pitch_position", 1);
    elbow = nh.advertise<std_msgs::Float64>("elbow_angle",1);
    std::cout << "Starting node 'arm_controller'" << "\n";

    ros::spin();

    return 0;
}
