#include "sensor_manager/sensor_manager.h"

using namespace sensor_manager;



int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_manager");

  // Create a SensorManager object
  SensorManager sm;

  //use single threaded spinner for global callback queue
   ros::spin();

//  ros::AsyncSpinner spinner(4); // Use 4 threads
//  spinner.start();
//  ros::waitForShutdown();

  return 0;
}
