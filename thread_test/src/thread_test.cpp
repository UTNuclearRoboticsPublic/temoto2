#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "thread_test/ThreadTestSrv.h"
#include <string>
#include <memory>
#include <mutex>
#include <stdio.h>
#include <csignal>
#include <sys/wait.h>
#include <algorithm>

bool loadCb(thread_test::ThreadTestSrv::Request& req, thread_test::ThreadTestSrv::Response& res)
{
  ROS_INFO("loadCb reached");
  // Name of the method, used for making debugging a bit simpler
  std::string prefix = __func__;

  // Get the service parameters

  ROS_INFO("%s Received a 'LoadProcess' service request: ...", prefix.c_str());

  // // Fork the parent process
  std::cout << "forking the process ..." << std::endl;
  pid_t PID = fork();

  // // Child process
  if (PID == 0)
  {
    // Execute the requested process
    std::cout << "executing program ..." << std::endl;
    // setpgid(0, 0);
    execlp("gnome-calculator", "gnome-calculator", (char*)NULL);
    /*  If execlp() is successful, we should not reach this next line. */
    std::cout << "The call to execlp() was not successful." << std::endl;
    exit(127);
  }

  // // Only parent gets here
  std::cout << "Process forked. Child PID: " << PID << std::endl;

  // Fill response
  res.pid.data = PID;
}

bool unloadCb(thread_test::ThreadTestSrv::Request& req, thread_test::ThreadTestSrv::Response& res)
{
  std::string prefix = __func__;
  ROS_INFO("%s Unload resource requested ...", prefix.c_str());

  // Lookup the requested process by its resource id.
  // Kill the process
  ROS_INFO("%s killing the child process (PID = %d)", prefix.c_str(), req.pid.data);

  int ret = kill(req.pid.data, SIGTERM);

  ROS_INFO("%s kill(SIGTERM) ret code: %d)", prefix.c_str(), ret);
  switch (ret)
  {
    case EINVAL:
      printf("An invalid signal was specified.\n");
      break;

    case EPERM:
      printf("The process does not have permission to send the signal to any of the target "
             "processes.\n");
      break;

    case ESRCH:
      printf("The  pid or process group does not exist.");
      break;
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "thread_test");
  ros::NodeHandle nh;

  ros::ServiceClient load_client = nh.serviceClient<thread_test::ThreadTestSrv>("load");

  ros::CallbackQueue load_cb_queue;
  ros::CallbackQueue unload_cb_queue;
  ros::AsyncSpinner load_spinner(1, &load_cb_queue);
  ros::AsyncSpinner unload_spinner(1, &unload_cb_queue);

  ros::AdvertiseServiceOptions load_service_opts =
      ros::AdvertiseServiceOptions::create<thread_test::ThreadTestSrv>(
          "load", &loadCb, ros::VoidPtr(), &load_cb_queue);
  ros::ServiceServer load_server = nh.advertiseService(load_service_opts);

  ros::AdvertiseServiceOptions unload_service_opts =
      ros::AdvertiseServiceOptions::create<thread_test::ThreadTestSrv>(
          "unload", &unloadCb, ros::VoidPtr(), &unload_cb_queue);
  ros::ServiceServer unload_server = nh.advertiseService(unload_service_opts);

  ROS_INFO("Node is good to go");

  // ros::AsyncSpinner spinner(4); // Use 4 threads
  // spinner.start();
  // ros::waitForShutdown();
  load_spinner.start();
  unload_spinner.start();
  // ros::spin();
  while (ros::ok())
  {
    ros::spinOnce();
    ros::Duration(2).sleep();
    thread_test::ThreadTestSrv srv;
    ROS_INFO("Calling...");
    if (load_client.call(srv))
    {
      ROS_INFO("OK");
    }
    else
    {
      ROS_WARN("FAILED");
    }
  }

  load_spinner.stop();
  unload_spinner.stop();
  return 0;
}
