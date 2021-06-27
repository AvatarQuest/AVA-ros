#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Vector3.h"
// #include "msg/Vector4.h";

#include <sstream>

ros::Publisher pan_topic;
ros::Publisher tilt_topic;

int map(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void vectorCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    double x = msg->x;
    double offset_x = x + 180 > 360 ? x + 180 - 360 : x + 180;
    double mapped_x = 180 - map(map(offset_x, 45, 315, 0, 270), 0, 270, 0, 180) - 22;

    double y = msg->y;
    double offset_y = y + 180 > 360 ? y + 180 - 360 : y + 180;
    double mapped_y = map(map(offset_y, 45, 315, 0, 270), 0, 270, 180, 0);

    ROS_INFO("Message received: (%.2f, %2f)", mapped_x, mapped_y);
    std_msgs::Int32 pan_msg;
    std_msgs::Int32 tilt_msg;
    
    tilt_msg.data = mapped_x;
    pan_msg.data = mapped_y;
    
    tilt_topic.publish(tilt_msg);
    pan_topic.publish(pan_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pantilt");
    ros::NodeHandle nodehandle;

    ros::Subscriber pantilt_topic = nodehandle.subscribe("pantilt", 1, vectorCallback);
    pan_topic = nodehandle.advertise<std_msgs::Int32>("pan", 1);
    tilt_topic = nodehandle.advertise<std_msgs::Int32>("tilt", 1);
    
    std::cout << "Started node: 'pantilt'" << std::endl;

    ros::spin();
    return 0;
}
