#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

//thumb: 15-60
//middle: 0-70

ros::Publisher claw_angle_pub;
void thumb_middle_CB(const geometry_msgs::Vector3::ConstPtr& msg) {
    ROS_INFO("THUMB_MIDDLE DATA: <%.2f, %.2f, %.2f>", msg->x, msg->y, msg->z);
    std_msgs::Float64 claw_msg;
    claw_msg.data = -1.5*(msg->x + msg->y - 65);
    claw_angle_pub.publish(claw_msg);
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "hand_controller_2");
    ros::NodeHandle nh;

    ros::Subscriber left_thumb_pos = nh.subscribe("thumb_middle", 1, thumb_middle_CB);
    claw_angle_pub = nh.advertise<std_msgs::Float64>("claw_angle", 1);

    std::cout << "Started node: 'hand controller 2' " << std::endl;
    ros::spin();
    return 0;
}
