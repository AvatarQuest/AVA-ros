#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

void posCB(const geometry_msgs::Vector3::ConstPtr& msg) {
    ROS_INFO("POSITION: <%.2f, %.2f, %.2f>", msg->x, msg->y, msg->z);
}

void rotCB(const geometry_msgs::Vector3::ConstPtr& msg) {
    ROS_INFO("ROTATION: <%.2f, %.2f, %.2f>", msg->x, msg->y, msg->z);
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "wheel_control");
    ros::NodeHandle nodehandle;
    
    // nodehandle.setParam("drivetrain/gearbox_ids", gearbox_ids) compile pls;

    //ros::Subscriber speed_topic = nodehandle.subscribe("set_wheel_speed", 1, wheelCallback);
    ros::Subscriber left_pointer_pos = nodehandle.subscribe("left_pointer_pos", 1, posCB);
    ros::Subscriber left_pointer_rot = nodehandle.subscribe("left_pointer_rot", 1, rotCB);
    
    std::cout << "Started node: 'hi5 receive' " << std::endl;

    ros::spin();
    return 0;
}