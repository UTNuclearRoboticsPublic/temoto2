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
#include <sys/syscall.h>


bool relayCb(thread_test::ThreadTestSrv::Request& req, thread_test::ThreadTestSrv::Response& res)
{
  pid_t tid = syscall(SYS_gettid);
  ROS_INFO("loadCb reached tid:%d", tid);
  // Name of the method, used for making debugging a bit simpler
  std::string prefix = __func__;

  // Get the service parameters

  ros::NodeHandle nh;
  ros::ServiceClient load_client = nh.serviceClient<thread_test::ThreadTestSrv>("load");
  ros::ServiceClient unload_client = nh.serviceClient<thread_test::ThreadTestSrv>("unload");
  thread_test::ThreadTestSrv srv;
  //if we have pid unload it, otherwise call load.
  if (!req.pid.data)
  {
    ROS_INFO("Calling load...");
    if (load_client.call(srv))
    {
      ROS_INFO("OK");
    }
    else
    {
      ROS_WARN("FAILED");
    }
  }
  else
  {
    ROS_INFO("Calling unload...");
    srv.request = req;
    if (unload_client.call(srv))
    {
      ROS_INFO("OK");
    }
    else
    {
      ROS_WARN("FAILED");
    }
  }
  std::cout << "Call returned: " << srv.response << std::endl;

  // relay response
  res = srv.response;
  return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "thread_relay");

  ros::NodeHandle nh;

  ros::CallbackQueue relay_cb_queue;
  ros::AsyncSpinner relay_spinner(1, &relay_cb_queue);
  ros::AsyncSpinner spinner(1);  // use async spinner for global callback queue

  ros::AdvertiseServiceOptions relay_service_opts =
      ros::AdvertiseServiceOptions::create<thread_test::ThreadTestSrv>(
          "relay", &relayCb, ros::VoidPtr(), &relay_cb_queue);
  ros::ServiceServer relay_server = nh.advertiseService(relay_service_opts);

  pid_t tid = syscall(SYS_gettid);
  ROS_INFO("Node is good to go. Main loop in thread %d", tid);

   spinner.start();
   relay_spinner.start();

   ros::waitForShutdown();

   relay_spinner.stop();
   return 0;
}
