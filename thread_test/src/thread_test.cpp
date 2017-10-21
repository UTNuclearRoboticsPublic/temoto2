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

void debugPid()
{
    printf("\tsession id: %u\n\
        proc parent id: %u\n\
        proc group id: %u\n\
        proc id: %u\n\
        thread id: %lu\n",
       getsid(0), getppid(), getpgid(0), getpid(), syscall(SYS_gettid));
}

pid_t xterm_pid_ = 0;

bool loadCb(thread_test::ThreadTestSrv::Request& req, thread_test::ThreadTestSrv::Response& res)
{
  pid_t tid = syscall(SYS_gettid);
  ROS_INFO("loadCb reached tid:%d", tid);
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
    // This is forked process
    std::cout << "This is forked child:" << std::endl;
    debugPid();
    setsid();
    setpgid(0, 0);
    std::cout << "after setsid():" << std::endl;
    debugPid();
    // Execute the requested process
    // execlp("gnome-calculator", "gnome-calculator", (char*)NULL);
    execlp("xterm", "xterm", (char*)NULL);
    /*  If execlp() is successful, we should not reach this next line. */
    std::cout << "The call to execlp() was not successful." << std::endl;
    exit(127);
  }

  // This is parent process
  std::cout << "Parent: fork() returned child PID: " << PID << std::endl;
  debugPid();
//    int stat;
//    pid_t pid = wait(&stat);
//    printf("pid %u, %u", pid, WSTOPSIG(stat));
  // Fill response
  res.pid.data = PID;
  return true;
}

bool unloadCb(thread_test::ThreadTestSrv::Request& req, thread_test::ThreadTestSrv::Response& res)
{
  std::string prefix = __func__;
  ROS_INFO("%s Unload resource requested ...", prefix.c_str());

  // Lookup the requested process by its resource id.
  // Kill the process
  ROS_INFO("%s SENDING KILL(SIGTERM) to PID %d", prefix.c_str(), req.pid.data);

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

  ros::ServiceClient load_client = nh.serviceClient<thread_test::ThreadTestSrv>("relay");

  ros::CallbackQueue load_cb_queue;
  ros::CallbackQueue unload_cb_queue;
  ros::AsyncSpinner load_spinner(1, &load_cb_queue);
  ros::AsyncSpinner unload_spinner(1, &unload_cb_queue);

//  ros::AdvertiseServiceOptions load_service_opts =
//      ros::AdvertiseServiceOptions::create<thread_test::ThreadTestSrv>(
//          "load", &loadCb, ros::VoidPtr(), &load_cb_queue);
//  ros::ServiceServer load_server = nh.advertiseService(load_service_opts);
  ros::ServiceServer load_server = nh.advertiseService("load", loadCb);

  ros::AdvertiseServiceOptions unload_service_opts =
      ros::AdvertiseServiceOptions::create<thread_test::ThreadTestSrv>(
          "unload", &unloadCb, ros::VoidPtr(), &unload_cb_queue);
  ros::ServiceServer unload_server = nh.advertiseService(unload_service_opts);

  ROS_INFO("This is main loop");
  debugPid();

   ros::AsyncSpinner spinner(1); // use async spinner for global callback queue
   spinner.start();
  load_spinner.start();
  unload_spinner.start();
  thread_test::ThreadTestSrv srv;
//   ros::waitForShutdown();
//    ROS_INFO("Calling without pid...");
//    if (load_client.call(srv))
//    {
//      ROS_INFO("OK");
//    }
//    else
//    {
//      ROS_WARN("FAILED");
//    }
//    srv.request.pid.data = srv.response.pid.data;
//    ros::Duration(5).sleep();
//
//    ROS_INFO("Calling with pid...");
//    if (load_client.call(srv))
//    {
//      ROS_INFO("OK");
//    }
//    else
//    {
//      ROS_WARN("FAILED");
//    }
  //    srv.request.pid.data = srv.response.pid.data;

  while (ros::ok())
  {
//    if (std::cin.get() != '\n')
//    {
//      ROS_INFO("Calling...");
//      if (load_client.call(srv))
//      {
//        ROS_INFO("OK");
//      }
//      else
//      {
//        ROS_WARN("FAILED");
//      }
//      srv.request.pid.data = srv.response.pid.data;
//
//      // ros::Duration(0.1).sleep();
//      //
//    }
//    ros::spinOnce();
    ros::Duration(1).sleep();

    if(xterm_pid_)
    {
      std::cout << "xterm is alive: " << xterm_pid_<<std::endl;
    }

  }

  load_spinner.stop();
  unload_spinner.stop();
  return 0;
}
