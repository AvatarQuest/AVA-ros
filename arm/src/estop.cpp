#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Vector3.h"
#include <sstream>

bool estop = false;
bool check = false;
bool started = false;

ros::Publisher drivetrain_topic;

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
        geometry_msgs::Vector3 zero;

        zero.x = 0;
        zero.y = 0;
        zero.z = 0;

        drivetrain_topic.publish(zero);
        drivetrain_topic.publish(zero);
        drivetrain_topic.publish(zero);

        ROS_ERROR("KILLING ALL NODES");
        drivetrain_topic.publish(zero);
        drivetrain_topic.publish(zero);
        drivetrain_topic.publish(zero);
        drivetrain_topic.publish(zero);
        drivetrain_topic.publish(zero);
        drivetrain_topic.publish(zero);
        drivetrain_topic.publish(zero);

        system("rosnode kill -a");
        system("rosnode kill -a");
        system("rosnode kill -a");
        system("rosnode kill -a");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "estop");
    ros::NodeHandle nodehandle;

    ros::Subscriber estop_topic = nodehandle.subscribe("estop", 10, estopCallback);
    drivetrain_topic = nodehandle.advertise<geometry_msgs::Vector3>("set_drivetrain_speed", 1);
    
    std::cout << "Started node: 'estop'" << std::endl;

    ros::Rate rate(1);

    while (ros::ok()) {
        if (!check && started) {
            ROS_ERROR("ESTOP SIGNAL RECIEVED, KILLING ROSBRIDGE, THEN ALL NODES");
            system("rosnode kill rosbridge_websocket");
    	    geometry_msgs::Vector3 zero;

	    zero.x = 0;
	    zero.y = 0;
	    zero.z = 0;

	    drivetrain_topic.publish(zero);
        drivetrain_topic.publish(zero);
        drivetrain_topic.publish(zero);
        drivetrain_topic.publish(zero);
        drivetrain_topic.publish(zero);
        drivetrain_topic.publish(zero);
        drivetrain_topic.publish(zero);
        drivetrain_topic.publish(zero);
        drivetrain_topic.publish(zero);

	    ROS_ERROR("KILLING ALL NODES");

	    system("rosnode kill -a");
        system("rosnode kill -a");
        system("rosnode kill -a");
        drivetrain_topic.publish(zero);
        drivetrain_topic.publish(zero);
        drivetrain_topic.publish(zero);
        drivetrain_topic.publish(zero);
        drivetrain_topic.publish(zero);
        drivetrain_topic.publish(zero);
        system("rosnode kill -a");
        system("rosnode kill -a");
        system("rosnode kill -a");

            return 0;
        }

        check = false;
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
