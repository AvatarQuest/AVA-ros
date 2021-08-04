#include "ros/ros.h"
#include <ros/callback_queue.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/hardware_interface.hpp>
// #include <various/shit/from_the/1950s.h>

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

  ROS_ERROR("MADE IT HERE");

  hardware_interface::JointHandle pos_handle_shoulder_pitch(jnt_state_interface.getHandle("shoulder_pitch"), &cmd[1]);
  jnt_pos_interface.registerHandle(pos_handle_shoulder_pitch);

  hardware_interface::JointHandle pos_handle_elbow(jnt_state_interface.getHandle("elbow"), &cmd[2]);
  jnt_pos_interface.registerHandle(pos_handle_elbow);

  hardware_interface::JointHandle pos_handle_wrist(jnt_state_interface.getHandle("wrist"), &cmd[3]);
  jnt_pos_interface.registerHandle(pos_handle_wrist);
  // Register the JointPositionInterface containing the read/write joints
  // with this robot's hardware_interface::RobotHW.
  registerInterface(&jnt_pos_interface);
}

RightArm::~RightArm() {}
  
bool RightArm::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {}

void RightArm::write() {
  for (auto c : pos) {
    ROS_WARN("COMMAND: %.2f", c);
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