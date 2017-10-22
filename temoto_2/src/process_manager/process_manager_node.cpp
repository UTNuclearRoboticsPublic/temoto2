#include "ros/ros.h"
#include "process_manager/process_manager.h"

//#include <sstream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "process_manager");

  // Create instance of process manager
  process_manager::ProcessManager pm;

  // set up ROS timer to update the process_manager
  ros::NodeHandle nh;
  ros::Timer timer =
      nh.createTimer(ros::Duration(1), &process_manager::ProcessManager::update, &pm);
  ros::spin();
  return 0;
}
