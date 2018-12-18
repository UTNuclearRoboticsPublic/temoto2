#include "ros/ros.h"
#include "temoto_er_manager/temoto_er_manager.h"

//#include <sstream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "temoto_er_manager");

  // Create instance of process manager
  temoto_er_manager::ERManager pm;

  // set up ROS timer to update the temoto_er_manager
  ros::NodeHandle nh;
  ros::Timer timer =
      nh.createTimer(ros::Duration(1), &temoto_er_manager::ERManager::update, &pm);
  ros::spin();
  return 0;
}
