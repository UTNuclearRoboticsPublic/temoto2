/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *          MASSIVE TODO: * CATCH ALL EXCEPTIONS !!!
 *                        * implement interprocess piping service
 *                          that starts streaming the std::out of
 *                          a requested process.
 *                        * organize your sh*t
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "core/common.h"
#include "process_manager/process_manager.h"

#include <stdio.h>
#include <csignal>
#include <sys/wait.h>
#include <algorithm>
#include <spawn.h>

namespace process_manager
{
ProcessManager::ProcessManager() : resource_manager_(srv_name::MANAGER, this)
{
  resource_manager_.addServer<temoto_2::LoadProcess>(srv_name::SERVER, &ProcessManager::loadCb,
                                                     &ProcessManager::unloadCb);
}

ProcessManager::~ProcessManager()
{
}

// Timer callback where running proceses are checked if they are operational
void ProcessManager::update(const ros::TimerEvent&)
{
  std::string prefix = node_name_ + "::" + __func__;

  // execute each process in loading_processes vector
  for (auto& srv : loading_processes_)
  {
    const std::string& action = srv.request.action;
    const std::string& package_name = srv.request.package_name;
    const std::string& executable = srv.request.executable;

    // Fork the parent process
    std::cout << "forking the process ..." << std::endl;
    pid_t PID = fork();

    // Child process
    if (PID == 0)
    {
      // Execute the requested process
      std::cout << "Child is executing a program ..." << std::endl;
      execlp(action.c_str(), action.c_str(), package_name.c_str(), executable.c_str(), (char*)NULL);
      return;
    }

    // Only parent gets here
    std::cout << "Parent: Process forked. Child PID: " << PID << std::endl;
    running_processes_.insert({PID, srv.response.rmp.resource_id});
  }
  loading_processes_.clear();






  // unload each process in unload_processes vector

  for (auto& proc : unloading_processes_)
  {
    // Kill the process
    ROS_INFO("%s killing the child process (PID = %d)", prefix.c_str(), proc.first);

    // TODO: Check the returned value
    int ret = kill(proc.first, SIGTERM);

    ROS_INFO("%s kill(SIGTERM) ret code: %d)", prefix.c_str(), ret);

    // Remove the process from the map
    running_processes_.erase(proc.first);
  }
  unloading_processes_.clear();

  


  // Check status of all running processes
  auto proc_it = running_processes_.begin();
  while (proc_it != running_processes_.end())
  {
    int status;
    int kill_response = waitpid(proc_it->first, &status, WNOHANG);

    //ROS_INFO("Resource_id '%d'(PID = %d) waitpid response = %d, status = %d\n", proc_it->second,
     //        proc_it->first, kill_response, status);

    // If the child process has stopped running,
    if (kill_response != 0)
    {
      ROS_ERROR("%s Process '%d'(PID = %d) has stopped, removing from process list and reporting",
                prefix.c_str(), proc_it->second, proc_it->first);

      // TODO: send error information to all related connections
      temoto_2::ResourceStatus srv;
      srv.request.resource_id = proc_it->second;
      srv.request.status_code = rmp::status_codes::FAILED;
      std::stringstream ss;
      ss << "The process with PID = ";
      ss << proc_it->first;
      ss << " was stopped.";
      srv.request.message = ss.str();

      resource_manager_.sendStatus(srv);

      // remove stopped process from the map
      running_processes_.erase(proc_it++);
    }
    else
    {
      proc_it++;
    }
  }
}

void ProcessManager::unloadCb(temoto_2::LoadProcess::Request& req,
                              temoto_2::LoadProcess::Response& res)
{
  std::string prefix = node_name_ + "::" + __func__;
  ROS_INFO("%s Unload resource requested ...", prefix.c_str());

  // Lookup the requested process by its resource id.
  pid_t active_pid = 0;
  bool pid_found = false;
  for (auto& proc : running_processes_)
  {
    if (proc.second == res.rmp.resource_id)
    {
      active_pid = proc.first;
      pid_found = true;
      unloading_processes_.insert({active_pid, res.rmp.resource_id});
      break;
    }
  }
  if (!pid_found)
  {
    ROS_WARN("%s unable to obtain PID for resource with id %ld. Resource might already be unloaded.", prefix.c_str(),
              res.rmp.resource_id);
    // TODO: throw exception
    return;
  }


  // Fill response
  res.rmp.code = 0;
  res.rmp.message = "Process queued for unloading.";

}

void ProcessManager::loadCb(temoto_2::LoadProcess::Request& req,
                            temoto_2::LoadProcess::Response& res)
{
  ROS_INFO("loadCb reached");
  // Name of the method, used for making debugging a bit simpler
  std::string prefix = node_name_ + "::" + __func__;

  // Get the service parameters
  const std::string& action = req.action;
  const std::string& package_name = req.package_name;
  const std::string& executable = req.executable;

  ROS_INFO("%s Received a 'LoadProcess' service request: %s ...", prefix.c_str(),
           executable.c_str());

  // Validate the action command.
  if (std::find(validActions.begin(), validActions.end(), action) == validActions.end())
  {
    ROS_INFO("%s Action '%s' is not supported ...", prefix.c_str(), action.c_str());
    // TODO THROW EXCEPTION
    return;
  }

  temoto_2::LoadProcess srv;
  srv.request = req;
  srv.response = res;
  loading_processes_.push_back(srv);

  // Fill response
  res.rmp.code = 0;
  res.rmp.message = "Command added to loading queue.";
}

}  // namespace process_manager
