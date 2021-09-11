#include <hand_controller/HandController.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <vector>

vector<double> left_thumb_state{ 1, 0, 0 };
vector<double> left_middle_state{ 0, 1, 0 };

double get_fingers_distance(vector<double> thumb, vector<double> middle) {
    return sqrt (
        pow(thumb[0] - middle[0],2) +
        pow(thumb[1] - middle[1],2) +
        pow(thumb[2] - middle[2],2)
        );
}
double get_claw_angle(double fingers_distance) {
    return fingers_distance * 10;
}
void pub_claw_angle_CB(const std_msgs::Float64::ConstPtr& msg){
    double claw_angle = get_claw_angle(get_fingers_distance(left_thumb_state, left_middle_state));
    msg.data = claw_angle;
}
void left_thumb_pos_CB(const geometry_msgs::Vector3::ConstPtr& msg) {
    ROS_INFO("LEFT THUMB POSITION: <%.2f, %.2f, %.2f>", msg->x, msg->y, msg->z);
    left_thumb_state[0] = msg->x;
    left_thumb_state[1] = msg->y;
    left_thumb_state[2] = msg->z;
}
void left_middle_pos_CB(const geometry_msgs::Vector3::ConstPtr& msg) {
    ROS_INFO("LEFT MIDDLE POSITION: <%.2f, %.2f, %.2f>", msg->x, msg->y, msg->z);
    left_middle_state[0] = msg->x;
    left_middle_state[1] = msg->y;
    left_middle_state[2] = msg->z;
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "hand_controller");
    ros::NodeHandle nh;
    
    ros::Subscriber left_thumb_pos = nodehandle.subscribe("left_thumb_pos", 1, left_pointer_pos_CB);
    ros::Subscriber left_middle_pos = nodehandle.subscribe("left_middle_pos", 1, left_middle_pos_CB);
    ros::Publisher claw_angle_pub = nodehandle.advertise<std_msgs::Float64>("/claw_angle", 1, pub_claw_angle_CB);

    std::cout << "Started node: 'hand controller' " << std::endl;
    ros::spin();
    return 0;
}
