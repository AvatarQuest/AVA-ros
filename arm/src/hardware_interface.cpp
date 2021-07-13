// Made by Alexander Nopes
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointPositionAction.h>
#include <trajectory_msgs/JointPosition.h>

class RobotTrajectoryFollower
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<control_msgs::FollowJointPositionAction> as_; 
  std::string action_name_;

public:

  RobotTrajectoryFollower(std::string name) :
    as_(nh_, name, false),
    action_name_(name)
  {
    //Register callback functions:
    as_.registerGoalCallback(boost::bind(&RobotTrajectoryFollower::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&RobotTrajectoryFollower::preemptCB, this));

    as_.start();
  }

  ~RobotTrajectoryFollower(void)//Destructor
  {
  }

  void goalCB()
  {
    // accept the new goal
    //goal_ = as_.acceptNewGoal()->samples;
    ROS_INFO("Accpeted the new goal");
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "hardware_interface");

  RobotTrajectoryFollower RobotTrajectoryFollower("/full_ctrl/joint_trajectory_action");
  ros::spin();

  return 0;
}