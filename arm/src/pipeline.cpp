#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
// #include "sensor_msgs/JointState.h"
#include "geometry_msgs/Vector3.h"
#include <map>

// #define PI 3.14159265359

// std::map<std::string, double> past_state;

ros::Publisher shoulder_yaw_topic;
ros::Publisher shoulder_pitch_topic;
ros::Publisher wrist_angle_topic;
ros::Publisher elbow_angle_topic;
ros::Publisher claw_angle_topic;

double shoulder_yaw_state = 1;
double shoulder_pitch_state = 1;
double wrist_angle_state = 0;
double elbow_angle_state = 0;
double claw_angle_state = 0;

void pipeline_callback(const geometry_msgs::Vector3::ConstPtr& msg) {
    // shoulder_yaw_state += (msg->x * 10/60) * 1400/360;
    // shoulder_pitch_state += (msg->y * 10/60) *1400/360;
    if (elbow_angle_state + (msg->z * 5/10) < 90 && elbow_angle_state + (msg->z) * 5/10 > -90) {
        elbow_angle_state += (msg->z * 5/10);
    }

    std_msgs::Float64 shoulder_yaw_message;
    std_msgs::Float64 shoulder_pitch_message;
    std_msgs::Float64 elbow_angle_message;

    shoulder_yaw_message.data = msg->x;
    shoulder_pitch_message.data = msg->y * 1;
    elbow_angle_message.data = elbow_angle_state + 180;

    if (shoulder_yaw_state != 0) shoulder_yaw_topic.publish(shoulder_yaw_message);
    if (shoulder_pitch_state != 0) shoulder_pitch_topic.publish(shoulder_pitch_message);
    elbow_angle_topic.publish(elbow_angle_message);

    shoulder_yaw_state = msg->x * 1;
    shoulder_pitch_state = msg->y;

    ROS_INFO("SHOULDER_YAW: %.2f", shoulder_yaw_state);
    ROS_INFO("SHOULDER_PITCH: %.2f", shoulder_pitch_state);
    ROS_INFO("ELBOW_ANGLE: %.2f", elbow_angle_state);
    // for (int i = 0; i <= sizeof(msg->position)/sizeof(msg->position[0]);i++) {
    //     if (past_state[msg->name[i]] != msg->position[i]) {
    //         past_state[msg->name[i]] = msg->position[i];     
    //         double angle = msg->position[i] * 180/PI;
    //         std::string name = msg->name[i];

    //         ROS_INFO("Joint: %s at %.5f", msg->name[i].c_str(), angle);

    //         std_msgs::Float64 angle_message;
            
    //         if (name == "shoulder_yaw") {
    //             angle_message.data = -angle*1400/360; //one encoder rotation is 1400 degrees
    //             // ROS_INFO("shoudler yaw: %.2f", -angle*1400/360);
    //             shoulder_yaw_topic.publish(angle_message);
    //         } else if (name == "shoulder_pitch") {
    //             angle_message.data = angle*1400/360;
    //             // ROS_INFO("shoudler pitch: %.2f", -angle*1400/360);
    //             shoulder_pitch_topic.publish(angle_message);
    //         } else if (name == "elbow") {
    //             angle_message.data = angle + 180;
    //             elbow_angle_topic.publish(angle_message);
    //         } else if (name == "wrist") {
    //             angle_message.data = angle;
    //             wrist_angle_topic.publish(angle_message);
    //         } 

    //     }
    // }

//     ROS_INFO("\n");
}

void wrist_callback(const std_msgs::Float64::ConstPtr& msg) {
    wrist_angle_state += msg->data * 5/10;

    std_msgs::Float64 wrist_angle_message;
    wrist_angle_message.data = wrist_angle_state;
    wrist_angle_topic.publish(wrist_angle_message);

    ROS_INFO("WRIST: %.2f", wrist_angle_state);
}

void claw_callback(const std_msgs::Int32::ConstPtr& msg) {
    if (claw_angle_state + (msg->data * 1) < 60 && claw_angle_state + (msg->data * 1) > -60) {
        claw_angle_state += msg->data * 1;
    }

    std_msgs::Float64 claw_angle_message;
    claw_angle_message.data = claw_angle_state;
    claw_angle_topic.publish(claw_angle_message);
    ROS_INFO("CLAW: %.2f", claw_angle_state);
}

int main(int argc, char **argv) {
    // past_state.insert(std::pair<std::string, double> ("shoulder_yaw", 0));
    // past_state.insert(std::pair<std::string, double> ("shouder_pitch", 0));
    // past_state.insert(std::pair<std::string, double> ("elbow", 0));
    // past_state.insert(std::pair<std::string, double> ("wrist", 0));
    
    ros::init(argc, argv, "arm_pipeline");
    ros::NodeHandle nodehandle;

    ros::Subscriber joint_state_topic = nodehandle.subscribe("arm_position", 1, pipeline_callback);
    ros::Subscriber wrist_input_topic = nodehandle.subscribe("wrist_input", 1, wrist_callback);
    ros::Subscriber claw_input_topic = nodehandle.subscribe("claw", 1, claw_callback);
    
    shoulder_yaw_topic = nodehandle.advertise<std_msgs::Float64>("shoulder_yaw", 10);
    shoulder_pitch_topic = nodehandle.advertise<std_msgs::Float64>("shoulder_pitch", 10);
    wrist_angle_topic = nodehandle.advertise<std_msgs::Float64>("wrist_angle", 10);
    elbow_angle_topic = nodehandle.advertise<std_msgs::Float64>("elbow_angle", 10);
    claw_angle_topic = nodehandle.advertise<std_msgs::Float64>("claw_angle", 10);
    
    std::cout << "Started node: 'arm_pipeline'" << "\n";
    ros::spin();
    return 0;
}
