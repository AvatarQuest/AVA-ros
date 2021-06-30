#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include <sstream>

bool estop = false;
bool check = false;
bool started = false;

ros::Publisher right_side_topic;
ros::Publisher left_side_topic;

void estopCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (!started) {
        ROS_INFO("USER CONNECTED");
    } 
    started = true;
    check = true;
    estop = msg->data;

    if (estop) {
        ROS_ERROR("ESTOP SIGNAL RECIEVED, KILLING ROSBRIDGE, THEN ALL NODES");
        system("rosnode kill rosbridge_websocket");
        std_msgs::Float64 right, left;

        right.data = 0;
        left.data = 0;

        right_side_topic.publish(right);
        left_side_topic.publish(left);

        ROS_ERROR("KILLING ALL NODES");

        system("rosnode kill -a");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "estop");
    ros::NodeHandle nodehandle;

    ros::Subscriber estop_topic = nodehandle.subscribe("estop", 10, estopCallback);
    right_side_topic = nodehandle.advertise<std_msgs::Float64>("right_side_speed", 1);
    left_side_topic = nodehandle.advertise<std_msgs::Float64>("left_side_speed", 1);
    
    std::cout << "Started node: 'estop'" << std::endl;

    ros::Rate rate(1);

    while (ros::ok()) {
        if (!check && started) {
            ROS_ERROR("ESTOP SIGNAL RECIEVED, KILLING ROSBRIDGE, THEN ALL NODES");
            system("rosnode kill rosbridge_websocket");
            std_msgs::Float64 right, left;

            right.data = 0;
            left.data = 0;

            right_side_topic.publish(right);
            left_side_topic.publish(left);

            ROS_ERROR("KILLING ALL NODES");

            system("rosnode kill -a");
            return 0;
        }

        check = false;
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
