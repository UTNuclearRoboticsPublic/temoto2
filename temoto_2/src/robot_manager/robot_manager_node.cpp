#include "robot_manager/robot_manager.h"

using namespace robot_manager;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_manager");

  // Create a SensorManager object
  RobotManager rm;

  // set up some robots
  rm.pkg_infos_.emplace_back(std::make_shared<PackageInfo>("task_robot", "ur5"));
  rm.pkg_infos_.back()->addLaunchable({ "ur5.launch", "" });

  rm.pkg_infos_.emplace_back(std::make_shared<PackageInfo>("task_robot", "ur3"));
  rm.pkg_infos_.back()->addLaunchable({ "ur3.launch", "" });

  //use single threaded spinner for global callback queue
   ros::spin();

  return 0;
}
