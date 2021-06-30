#include <vector>
#include "ros/ros.h"
#include "drivetrain/Motor.h"
#include "geometry_msgs/Vector3.h"
//#include <signal.h>

#include <sstream>

double map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

ros::Publisher pwm_topic;
const std::vector<int> drivetrain_ids = {3, 5, 6, 7, 8, 9, 10, 11};
int drivetrain_states[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
void closeNode(int sig)
{
  for (const int &id : drivetrain_ids) {
    geometry_msgs::Vector3 speed_msg;
    speed_msg.x = id;
    speed_msg.y = 187;
    speed_msg.z = 0;

    pwm_topic.publish(speed_msg);
  }
  ros::shutdown();
}

void speedCallback(const drivetrain::Motor::ConstPtr& msg) {
    geometry_msgs::Vector3 speed_msg;

    int id = msg->id;
    int speed = msg->speed;
    if (abs(speed) > 65) {
      if (speed > 0){
        speed = 65;
        ROS_INFO("setting speed to 100");
      } else {
        speed = -65;
        ROS_INFO("setting speed to -100");
      }
    }
    

    //double speed = abs(msg->speed) > 100 ? msg->speed > 0 ? 100 : -100 : msg->speed;
    //speed = map(speed, -100, 100, -65, 65);
    //speed = speed > 0 ? floor(speed) : ceil(speed); 

    int pwm;
    if (speed == 0) {
      pwm = 187;
    } else if (speed > 0) {
      pwm = 190 + speed;
    } else if (speed < 0) {
      pwm = 184 + speed; // because speed is negative here
    }

    if (drivetrain_states[id] == 0) {
        drivetrain_states[id] = pwm;
    } else if (drivetrain_states[id] == pwm) {
        return;
    }
    drivetrain_states[id] = pwm;
    
    ROS_INFO("ID: %d, SPEED: %d, PWM: %d", id, msg->speed, pwm);
    
    speed_msg.x = id;
    speed_msg.y = pwm;
    speed_msg.z = 0;

    pwm_topic.publish(speed_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "speed_control");
    ros::NodeHandle nodehandle;
    
    nodehandle.setParam("drivetrain/motor_ids", drivetrain_ids);
    nodehandle.setParam("drivetrain/motor_count", int(sizeof(drivetrain_ids)/sizeof(drivetrain_ids[0])));

    ros::Subscriber speed_topic = nodehandle.subscribe("set_motor_speed", 20, speedCallback);
    pwm_topic = nodehandle.advertise<geometry_msgs::Vector3>("set_motor_pwm", 20);
    
    std::cout << "Started node: 'speed_control'" << std::endl;

    //signal(SIGINT, closeNode);

    ros::spin();
    return 0;
}
