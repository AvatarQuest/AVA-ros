
#include <moveit_servo/servo.h>

namespace
{
constexpr char LOGNAME[] = "servo_server";
constexpr char ROS_THREADS = 8;

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, LOGNAME);
  ros::AsyncSpinner spinner(ROS_THREADS);
  spinner.start();

  ros::NodeHandle nh;

  // Load the planning scene monitor
  auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  // Start the planning scene monitor
  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
  planning_scene_monitor->startStateMonitor();

  // Create the servo server
  moveit_servo::Servo servo(nh, planning_scene_monitor);

  // Start the servo server (runs in the ros spinner)
  servo.start();

  // Wait for ros to shutdown
  ros::waitForShutdown();

  // Stop the servo server
  // servo.stop();

  return 0;
}
