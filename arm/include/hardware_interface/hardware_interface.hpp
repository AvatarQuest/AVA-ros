#include "ros/ros.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

namespace right_arm {
  class RightArm : public hardware_interface::RobotHW, public hardware_interface::HardwareInterface {
    public:
      RightArm();

      ~RightArm();

      bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh);

      void write();
      
      void read();
    
    private:
      hardware_interface::JointStateInterface jnt_state_interface;
      hardware_interface::PositionJointInterface jnt_pos_interface;

      double cmd[4];
      double pos[4];
      double vel[4];
      double eff[4];

      double cmd_state[4];

      ros::NodeHandle robot_hw_nh;
      ros::Publisher shoulder_yaw_topic;
      ros::Publisher shoulder_pitch_topic;
      ros::Publisher wrist_angle_topic;
      ros::Publisher elbow_angle_topic;
      ros::Publisher claw_angle_topic;
  };
}
