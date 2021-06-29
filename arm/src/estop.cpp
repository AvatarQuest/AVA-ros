#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <sstream>

bool estop = false;
bool check = false;
bool started = false;

void estopCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (!started) {
        ROS_INFO("USER CONNECTED");
    } 
    started = true;
    check = true;
    estop = msg->data;

    if (estop) {
        ROS_ERROR("ESTOP SIGNAL RECIEVED, KILLING ALL NODES");
        system("rosnode kill -a");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "estop");
    ros::NodeHandle nodehandle;

    ros::Subscriber estop_topic = nodehandle.subscribe("estop", 10, estopCallback);
    
    std::cout << "Started node: 'estop'" << std::endl;

    ros::Rate rate(1);

    while (ros::ok()) {
        if (!check && started) {
            ROS_ERROR("CHECK FAILED, KILLING ALL NODES");
            system("rosnode kill -a");
            return 0;
        }

        check = false;
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
