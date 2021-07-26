#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
 
class RightArm : public hardware_interface::RobotHW
{
public:
  RightArm() 
  { 
    // Initialization of the robot's resources (joints, sensors, actuators) and
    // interfaces can be done here or inside init().
    // E.g. parse the URDF for joint names & interfaces, then initialize them
  }
 
  bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
  {
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
 
    hardware_interface::JointHandle pos_handle_shoulder_pitch(jnt_state_interface.getHandle("shoulder_pitch"), &cmd[1]);
    jnt_pos_interface.registerHandle(pos_handle_shoulder_pitch);
 
    hardware_interface::JointHandle pos_handle_elbow(jnt_state_interface.getHandle("elbow"), &cmd[2]);
    jnt_pos_interface.registerHandle(pos_handle_elbow);
 
    hardware_interface::JointHandle pos_handle_wrist(jnt_state_interface.getHandle("wrist"), &cmd[3]);
    jnt_pos_interface.registerHandle(pos_handle_wrist);
    // Register the JointPositionInterface containing the read/write joints
    // with this robot's hardware_interface::RobotHW.
    registerInterface(&jnt_pos_interface);
 
    return true;
  }
 
private:
  // hardware_interface::JointStateInterface gives read access to all joint values 
  // without conflicting with other controllers.
  hardware_interface::JointStateInterface jnt_state_interface;
  // hardware_interface::PositionJointInterface inherits from 
  // hardware_interface::JointCommandInterface and is used for reading and writing
  // joint positions. Because this interface reserves the joints for write access,
  // conflicts with other controllers writing to the same joints might occure.
  // To only read joint positions, avoid conflicts using 
  // hardware_interface::JointStateInterface.
  hardware_interface::PositionJointInterface jnt_pos_interface;
 
  // Data member array to store the controller commands which are sent to the 
  // robot's resources (joints, actuators)
  double cmd[4];
 
  // Data member arrays to store the state of the robot's resources (joints, sensors)
  double pos[4];
  double vel[4];
  double eff[4];
};