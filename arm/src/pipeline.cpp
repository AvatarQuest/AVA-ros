#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <map>

#define PI 3.14159265359

std::map<std::string, double> past_state;


ros::Publisher shoulder_yaw_topic;
ros::Publisher shoulder_pitch_topic;
ros::Publisher wrist_angle_topic;
ros::Publisher elbow_angle_topic;


void pipeline_callback(const sensor_msgs::JointState::ConstPtr& msg) {
    for (int i = 0; i <= sizeof(msg->position)/sizeof(msg->position[0]);i++) {
        if (past_state[msg->name[i]] != msg->position[i]) {
            past_state[msg->name[i]] = msg->position[i];     
            double angle = msg->position[i] * 180/PI;
            std::string name = msg->name[i];

            ROS_INFO("Joint: %s at %.5f", msg->name[i].c_str(), angle);

            std_msgs::Float64 angle_message;
            
            if (name == "shoulder_yaw") {
                angle_message.data = -angle*1400/360; //one encoder rotation is 1400 degrees
                // ROS_INFO("shoudler yaw: %.2f", -angle*1400/360);
                shoulder_yaw_topic.publish(angle_message);
            } else if (name == "shoulder_pitch") {
                angle_message.data = angle*1400/360;
                // ROS_INFO("shoudler pitch: %.2f", -angle*1400/360);
                shoulder_pitch_topic.publish(angle_message);
            } else if (name == "elbow") {
                angle_message.data = angle + 180;
                elbow_angle_topic.publish(angle_message);
            } else if (name == "wrist") {
                angle_message.data = angle;
                wrist_angle_topic.publish(angle_message);
            } 

        }
    }

//     ROS_INFO("\n");
}

int main(int argc, char **argv) {
    past_state.insert(std::pair<std::string, double> ("shoulder_yaw", 0));
    past_state.insert(std::pair<std::string, double> ("shouder_pitch", 0));
    past_state.insert(std::pair<std::string, double> ("elbow", 0));
    past_state.insert(std::pair<std::string, double> ("wrist", 0));
    
    ros::init(argc, argv, "arm_pipeline");
    ros::NodeHandle nodehandle;

    ros::Subscriber joint_state_topic = nodehandle.subscribe("joint_states", 1, pipeline_callback);
    
    shoulder_yaw_topic = nodehandle.advertise<std_msgs::Float64>("shoulder_yaw", 10);
    shoulder_pitch_topic = nodehandle.advertise<std_msgs::Float64>("shoulder_pitch", 10);
    wrist_angle_topic = nodehandle.advertise<std_msgs::Float64>("wrist_angle", 10);
    elbow_angle_topic = nodehandle.advertise<std_msgs::Float64>("elbow_angle", 10);
    
    std::cout << "Started node: 'arm_pipeline'" << "\n";
    ros::spin();
    return 0;
}
