#include "algorithm_manager/algorithm_manager.h"

using namespace algorithm_manager;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "algorithm_manager");

  // Create a AlgorithmManager object
  AlgorithmManager am;

  //use single threaded spinner for global callback queue
  ros::spin();

//  ros::AsyncSpinner spinner(4); // Use 4 threads
//  spinner.start();
//  ros::waitForShutdown();

  return 0;
}
