/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *          MASSIVE TODO: * CATCH ALL EXCEPTIONS !!!
 *                        * implement interprocess piping service
 *                          that starts streaming the std::out of
 *                          a requested process.
 *                        * organize your sh*t
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "process_manager/process_manager.h"

#include <stdio.h>
#include <csignal>
#include <sys/wait.h>
#include <algorithm>
#include <spawn.h>
#include <regex>

namespace process_manager
{
ProcessManager::ProcessManager() : resource_manager_(srv_name::MANAGER, this)
{
  class_name_ = __func__;
  subsystem_name_ = "process_manager";
  subsystem_code_ = error::Subsystem::ROBOT_MANAGER;
  log_group_ = "process_manager";
  error_handler_ = error::ErrorHandler(subsystem_code_, log_group_);

  resource_manager_.addServer<temoto_2::LoadProcess>(srv_name::SERVER, &ProcessManager::loadCb,
                                                     &ProcessManager::unloadCb);
  TEMOTO_INFO("Process manager is ready.");
}

ProcessManager::~ProcessManager()
{
}

// Timer callback where running proceses are checked if they are operational
void ProcessManager::update(const ros::TimerEvent&)
{
  // execute each process in loading_processes vector
  waitForLock(running_mutex_);
  waitForLock(loading_mutex_);
  for (auto& srv : loading_processes_)
  {
    const std::string& package_name = srv.request.package_name;
    const std::string& executable = srv.request.executable;
    const std::string& args = srv.request.args;

    std::string cmd = "";

    if (srv.request.action == action::ROS_EXECUTE)
    {
      if (srv.request.ros_namespace != "")
      {
        cmd += "ROS_NAMESPACE=" + common::getAbsolutePath(srv.request.ros_namespace) + " ";
      }
      std::regex rx(".*\\.launch$");
      cmd += (std::regex_match(executable, rx)) ? "roslaunch " : "rosrun ";
      cmd += package_name + " " + executable + " " + args;
    }
    else if (srv.request.action == action::SYS_EXECUTE)
    {
      cmd += executable + " " + args;
    }

    // Fork the parent process
    TEMOTO_DEBUG("Forking the process.");
    pid_t pid = fork();

    // Child process
    if (pid == 0)
    {
      // Execute the requested process
      //  std::cout << "Child is executing a program ..." << std::endl;
      execlp("/bin/bash", "/bin/bash", "-c", cmd.c_str() , (char*)NULL);
      return;
    }

    // Only parent gets here
    TEMOTO_DEBUG("Child %d forked.", pid);
    running_processes_.insert({ pid, srv });
  }
  loading_processes_.clear();
  loading_mutex_.unlock();

  // unload each process in unload_processes vector
  waitForLock(unloading_mutex_);
  for (auto pid : unloading_processes_)
  {
    // consistency check, ensure that pid is still in running_processes
    auto proc_it = running_processes_.find(pid);
    if (proc_it != running_processes_.end())
    {
      // Kill the process
      TEMOTO_DEBUG("Sending kill(SIGTERM) to %d", pid);

      int ret = kill(pid, SIGTERM);
      TEMOTO_DEBUG("kill(SIGTERM) returned: %d", ret);
      // TODO: Check the returned value

      // Remove the process from the map
      running_processes_.erase(proc_it);
    }
    else
    {
      TEMOTO_DEBUG("Unable to normally unload reource with pid: %d. Resource not running any more.", pid);
    }
  }
  unloading_processes_.clear();

  // Check the status of all running processes
  // cache all to statuses before actual sending, so we can release the running_mutex.
  std::vector<temoto_2::ResourceStatus> statuses_to_send; 
  auto proc_it = running_processes_.begin();
  while (proc_it != running_processes_.end())
  {
    int status;
    int kill_response = waitpid(proc_it->first, &status, WNOHANG);
    // If the child process has stopped running,
    if (kill_response != 0)
    {
      TEMOTO_ERROR("Process %d ('%s' '%s' '%s') has stopped.", proc_it->first,
                   proc_it->second.request.action.c_str(),
                   proc_it->second.request.package_name.c_str(),
                   proc_it->second.request.executable.c_str());

      // TODO: send error information to all related connections
      temoto_2::ResourceStatus srv;
      srv.request.resource_id = proc_it->second.response.rmp.resource_id;
      srv.request.status_code = rmp::status_codes::FAILED;
      std::stringstream ss;
      ss << "The process with pid '" << proc_it->first << "' has stopped.";
      srv.request.message = ss.str();

      // store statuses to send
      statuses_to_send.push_back(srv);
      
      // Remove the process from the map
      proc_it = running_processes_.erase(proc_it);
    }
    else
    {
      proc_it++;
    }
  }
  running_mutex_.unlock();
  unloading_mutex_.unlock();

  for (auto& srv : statuses_to_send)
  {
    resource_manager_.sendStatus(srv);

    // TODO: Normally unload command should come from upper chain, howerver, when sending status us unsucessful, we should unload the resource manually?
   // running_processes_.erase(proc_it++);
  }
}

void ProcessManager::loadCb(temoto_2::LoadProcess::Request& req,
                            temoto_2::LoadProcess::Response& res)
{
  // Validate the action command.
  if (req.action == action::ROS_EXECUTE) //|| action == action::SYS_EXECUTE)
  {
    TEMOTO_DEBUG("adding '%s' '%s' '%s' to the loading queue.", req.action.c_str(),
                 req.package_name.c_str(), req.executable.c_str());

    temoto_2::LoadProcess srv;
    srv.request = req;
    srv.response = res;

    loading_mutex_.lock();
    loading_processes_.push_back(srv);
    loading_mutex_.unlock();

    // Fill response
    res.rmp.code = 0;
    res.rmp.message = "Request added to the loading queue.";
  }
  else
  {
    res.rmp.code = -1;
    res.rmp.message = "Action not supported by the process manager.";
    TEMOTO_ERROR("Action '%s' is not supported.", req.action.c_str());
    return;  // TODO THROW EXCEPTION TO RMP?
  }
}

void ProcessManager::unloadCb(temoto_2::LoadProcess::Request& req,
                              temoto_2::LoadProcess::Response& res)
{
  TEMOTO_DEBUG("Unloading resource with id '%ld' ...", res.rmp.resource_id);

  // Lookup the requested process by its resource id.
  waitForLock(running_mutex_);
  auto proc_it =
      std::find_if(running_processes_.begin(), running_processes_.end(),
                   [&](const std::pair< pid_t, temoto_2::LoadProcess>& p) -> bool { return p.second.request == req; });
  if (proc_it != running_processes_.end())
  {
    unloading_processes_.push_back(proc_it->first);
    res.rmp.code = 0;
    res.rmp.message = "Resource added to unload queue.";
    TEMOTO_DEBUG("Resource with id '%ld' added to unload queue.", res.rmp.resource_id);
  }
  else
  {
    TEMOTO_ERROR("Unable to unload reource with resource_id: %ld. Resource is not running.", res.rmp.resource_id);
    // Fill response
    res.rmp.code = rmp::status_codes::FAILED;
    res.rmp.message = "Resource is not running. Unable to unload.";
  }
  running_mutex_.unlock();

  unloading_mutex_.unlock();
}

  void ProcessManager::waitForLock(std::mutex& m)
  {
    while (!m.try_lock())
    {
      TEMOTO_DEBUG("Waiting for lock()");
      ros::Duration(0.1).sleep();  // sleep for few ms
    }
  }
}  // namespace process_manager
