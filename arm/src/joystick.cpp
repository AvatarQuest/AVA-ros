#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
 
geometry_msgs::Pose target_pose;
static const std::string PLANNING_GROUP = "right_arm"; 
moveit::planning_interface::MoveGroupInterface* move_group;

void joystickCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    double x = msg->x;
    double y = msg->y;
    double z = msg->z;

    if (x == 0 && y == 0 && z == 0) {
        return;
    }
    target_pose.position.x = target_pose.position.x + x * 0.09;
    target_pose.position.y = target_pose.position.y + y * 0.09;
    target_pose.position.z = target_pose.position.z + z * 0.09;
    
    move_group->setStartStateToCurrentState();
    move_group->setApproximateJointValueTarget(target_pose);
    move_group->asyncMove();
    
    ROS_WARN("ARM POSITION: (%.3f, %.3f, %.3f)", target_pose.position.x, target_pose.position.y, target_pose.position.z);
}
 
 int main(int argc, char** argv) {
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    
    move_group = (new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP));

    target_pose = move_group->getPoseTarget().pose;
    target_pose.orientation.w = 0;
    move_group->setApproximateJointValueTarget(target_pose);
    move_group->asyncMove();

    move_group->setGoalOrientationTolerance(3.14);

    ros::Subscriber joystick_topic = node_handle.subscribe("arm_position", 1, joystickCallback);
    ros::waitForShutdown();
 }
