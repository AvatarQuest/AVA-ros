#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

ros::Publisher servo_server;

void left_thumb_pos_CB(const geometry_msgs::Vector3::ConstPtr& msg) { ROS_INFO("LEFT THUMB POSITION: <%.2f, %.2f, %.2f>", msg->x, msg->y, msg->z);
}
void left_thumb_rot_CB(const geometry_msgs::Vector3::ConstPtr& msg) {
    ROS_INFO("LEFT THUMB ROTATION: <%.2f, %.2f, %.2f>", msg->x, msg->y, msg->z);
}
void left_pointer_pos_CB(const geometry_msgs::Vector3::ConstPtr& msg) {
    ROS_INFO("LEFT POINTER POSITION: <%.2f, %.2f, %.2f>", msg->x, msg->y, msg->z);
}
void left_pointer_rot_CB(const geometry_msgs::Vector3::ConstPtr& msg) {
    ROS_INFO("LEFT POINTER ROTATION: <%.2f, %.2f, %.2f>", msg->x, msg->y, msg->z);
}
void left_middle_pos_CB(const geometry_msgs::Vector3::ConstPtr& msg) {
    ROS_INFO("LEFT MIDDLE POSITION: <%.2f, %.2f, %.2f>", msg->x, msg->y, msg->z);
}
void left_middle_rot_CB(const geometry_msgs::Vector3::ConstPtr& msg) {
    ROS_INFO("LEFT MIDDLE ROTATION: <%.2f, %.2f, %.2f>", msg->x, msg->y, msg->z);
}
void left_ring_pos_CB(const geometry_msgs::Vector3::ConstPtr& msg) {
    ROS_INFO("LEFT RING POSITION: <%.2f, %.2f, %.2f>", msg->x, msg->y, msg->z);
}
void left_ring_rot_CB(const geometry_msgs::Vector3::ConstPtr& msg) {
    ROS_INFO("LEFT RING ROTATION: <%.2f, %.2f, %.2f>", msg->x, msg->y, msg->z);
}
void left_pinky_pos_CB(const geometry_msgs::Vector3::ConstPtr& msg) {
    ROS_INFO("LEFT PINKY POSITION: <%.2f, %.2f, %.2f>", msg->x, msg->y, msg->z);
}
void left_pinky_rot_CB(const geometry_msgs::Vector3::ConstPtr& msg) {
    ROS_INFO("LEFT PINKY ROTATION: <%.2f, %.2f, %.2f>", msg->x, msg->y, msg->z);
}

void delta_hand_CB(const geometry_msgs::Twist::ConstPtr& msg) {
    ROS_INFO("ARM POSITION: <%.5f, %.5f, %.5f>", msg->linear.x, msg->linear.y, msg->linear.z);
    geometry_msgs::TwistStamped stamped;
    stamped.twist = *msg;
    stamped.header.stamp = ros::Time::now();

    servo_server.publish(stamped);
}

void wrist_input_CB(const std_msgs::Float64::ConstPtr& msg) {
    ROS_INFO("WRIST INPUT: %.2f", msg->data);
}
void claw_CB(const std_msgs::Int32::ConstPtr& msg) {
    ROS_INFO("CLAW: %.2f", msg->data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "hi5_receive");
    ros::NodeHandle nodehandle;
    
    servo_server = nodehandle.advertise<geometry_msgs::TwistStamped>("servo_server/delta_twist_cmds", 2);
   ros::Subscriber left_thumb_pos = nodehandle.subscribe("left_thumb_pos", 1, left_pointer_pos_CB);
   ros::Subscriber left_thumb_rot = nodehandle.subscribe("left_thumb_rot", 1, left_pointer_rot_CB);
   ros::Subscriber left_pointer_pos = nodehandle.subscribe("left_pointer_pos", 1, left_pointer_pos_CB);
   ros::Subscriber left_pointer_rot = nodehandle.subscribe("left_pointer_rot", 1, left_pointer_rot_CB);
   ros::Subscriber left_middle_pos = nodehandle.subscribe("left_middle_pos", 1, left_middle_pos_CB);
   ros::Subscriber left_middle_rot = nodehandle.subscribe("left_middle_rot", 1, left_middle_rot_CB);
   ros::Subscriber left_ring_pos = nodehandle.subscribe("left_ring_pos", 1, left_ring_pos_CB);
   ros::Subscriber left_ring_rot = nodehandle.subscribe("left_ring_rot", 1, left_ring_rot_CB);
   ros::Subscriber left_pinky_pos = nodehandle.subscribe("left_pinky_pos", 1, left_pinky_pos_CB);
   ros::Subscriber left_pinky_rot = nodehandle.subscribe("left_pinky_rot", 1, left_pinky_rot_CB);
    ros::Subscriber arm_velocity = nodehandle.subscribe("delta_hand", 1, delta_hand_CB);
    ros::Subscriber wrist_input = nodehandle.subscribe("wrist_input", 1, wrist_input_CB);
    std::cout << "Started node: 'hi5 receive' " << std::endl;

    ros::spin();
    return 0;
}
