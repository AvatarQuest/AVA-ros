#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <ros/callback_queue.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/hardware_interface.hpp>
// #include <various/shit/from_the/1950s.h>
#define PI 3.141592
using namespace right_arm; 

RightArm::RightArm() {
  // Create a JointStateHandle for each joint and register them with the 
  // JointStateInterface.
  hardware_interface::JointStateHandle state_handle_shoulder_yaw("shoulder_yaw", &pos[0], &vel[0], &eff[0]);
  jnt_state_interface.registerHandle(state_handle_shoulder_yaw);

  hardware_interface::JointStateHandle state_handle_shoulder_pitch("shoulder_pitch", &pos[1], &vel[1], &eff[1]);
  jnt_state_interface.registerHandle(state_handle_shoulder_pitch);

  hardware_interface::JointStateHandle state_handle_elbow("elbow", &pos[2], &vel[2], &eff[2]);
  jnt_state_interface.registerHandle(state_handle_elbow);

  hardware_interface::JointStateHandle state_handle_wrist("wrist", &pos[3], &vel[3], &eff[3]);
  jnt_state_interface.registerHandle(state_handle_wrist);
  // Register the JointStateInterface containing the read only joints
  // with this robot's hardware_interface::RobotHW.
  registerInterface(&jnt_state_interface);
  // Create a JointHandle (read and write) for each controllable joint
  // using the read-only joint handles within the JointStateInterface and 
  // register them with the JointPositionInterface.
  hardware_interface::JointHandle pos_handle_shoulder_yaw(jnt_state_interface.getHandle("shoulder_yaw"), &cmd[0]);
  jnt_pos_interface.registerHandle(pos_handle_shoulder_yaw);

  // ROS_ERROR("MADE IT HERE");

  hardware_interface::JointHandle pos_handle_shoulder_pitch(jnt_state_interface.getHandle("shoulder_pitch"), &cmd[1]);
  jnt_pos_interface.registerHandle(pos_handle_shoulder_pitch);

  hardware_interface::JointHandle pos_handle_elbow(jnt_state_interface.getHandle("elbow"), &cmd[2]);
  jnt_pos_interface.registerHandle(pos_handle_elbow);

  hardware_interface::JointHandle pos_handle_wrist(jnt_state_interface.getHandle("wrist"), &cmd[3]);
  jnt_pos_interface.registerHandle(pos_handle_wrist);
  // Register the JointPositionInterface containing the read/write joints
  // with this robot's hardware_interface::RobotHW.
  registerInterface(&jnt_pos_interface);

  shoulder_yaw_topic = robot_hw_nh.advertise<std_msgs::Int32>("set_right_arm_yaw_position", 5);
  shoulder_pitch_topic = robot_hw_nh.advertise<std_msgs::Int32>("set_right_arm_pitch_position", 5);
  wrist_angle_topic = robot_hw_nh.advertise<std_msgs::Int32>("wrist_angle", 5);
  elbow_angle_topic = robot_hw_nh.advertise<std_msgs::Int32>("elbow_angle", 5);
  // claw_angle_topic = robot_hw_nh.advertise<std_msgs::Int32>("claw_angle", 5);
}

RightArm::~RightArm() {}
  
bool RightArm::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
    this->robot_hw_nh = robot_hw_nh;
    return true;
}

void RightArm::write() {
  for (int i = 0; i < sizeof(cmd)/sizeof(cmd[0]); i++) {
    std_msgs::Int32 motor_msg;
    int command = 0;
    switch(i) { // best practice
      case 0:
        command = int((cmd[i] * 180/PI) + 0.5);

        if(command != cmd_state[0]) {
          motor_msg.data = command;
          shoulder_yaw_topic.publish(motor_msg);
          //shoulder yaw
          cmd_state[0] = command;
          ROS_INFO("shoulder yaw: %.2f",cmd[0] * 180/PI);
        }
        break;
      case 1:
        command = int((cmd[i] * 180/PI) + 0.5);
        if(command != cmd_state[1]) {
          motor_msg.data = command;
          shoulder_pitch_topic.publish(motor_msg);
          //shoulder yaw
          cmd_state[1] = command;
          ROS_INFO("shoulder pitch: %.2f",cmd[1] * 180/PI);
        }
        break;
      case 2:
        command = int((cmd[i] * 180/PI) + 0.5);
        if(command != cmd_state[2]) {
          motor_msg.data = command;
          elbow_angle_topic.publish(motor_msg);
          //shoulder yaw
          cmd_state[2] = command;
          ROS_INFO("elbow: %.2f", cmd[2] * 180/PI);
        }
        break;
      case 3:
        //wrist
        command = int((cmd[i] * 180/PI) + 0.5);

        if(command != cmd_state[3]) {
          motor_msg.data = command;
          wrist_angle_topic.publish(motor_msg);
          //shoulder yaw
          cmd_state[3] = command;
          ROS_INFO("wrist: %.2f",cmd[3] * 180/PI);
        }
       break; 
      default:
        break;
    }   
  }
}

void RightArm::read() {
  for (int i = 0; i < (sizeof(cmd) / sizeof(cmd[0])); i++) {
    pos[i] = cmd[i];
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "right_arm_hardware_interface");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  RightArm arm;

  controller_manager::ControllerManager cm(&arm,nh);
  spinner.start();
  ros::Rate rate(50);

  while (ros::ok()) {
     arm.read();
     cm.update(ros::Time::now(), rate.expectedCycleTime());
     arm.write();
     rate.sleep();
  }

  spinner.stop();

}