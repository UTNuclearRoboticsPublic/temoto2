#include "robot_manager/robot_manager.h"

using namespace robot_manager;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_manager");

  // Create a SensorManager object
  RobotManager rm;

  //use single threaded spinner for global callback queue
   ros::spin();

  return 0;
}
