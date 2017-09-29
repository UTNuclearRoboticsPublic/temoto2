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
#include "node_manager/node_manager.h"

#include <stdio.h>
#include <csignal>
#include <sys/wait.h>
#include <algorithm>

namespace node_manager
{

NodeManager::NodeManager():resource_manager_(this)
{
	resource_manager_.addServer<temoto_2::LoadResource, temoto_2::UnloadResource>(
			"node_manager_server", 
			&NodeManager::loadCb,
			&NodeManager::unloadCb);
}

NodeManager::~NodeManager()
{}


// Timer callback where running proceses are checked if they are operational
void NodeManager::update(const ros::TimerEvent&)
{
	//auto find_it = std::find(running_processes_.begin(), running_processes_.end(), res.resource_id);
	//if(find_it == running_processes_.end())
	//{
	//	ROS_ERROR("%s unable to obtain PID for resource with id %d", res.resource_id);
	//	return false;
	//}

	std::string prefix = node_name_ + "::" + __func__;

	// Run through the list of running processes
	auto proc_it = running_processes_.begin();
	while (proc_it != running_processes_.end())
	{
		int status;
		int kill_response = waitpid(proc_it->first, &status, WNOHANG);

		ROS_INFO("Resource_id '%d'(PID = %d) waitpid response = %d, status = %d\n",
				proc_it->second, proc_it->first,
				kill_response, status);

		// If the child process has stopped running,
		if(kill_response != 0)
		{
			ROS_ERROR("%s Process '%d'(PID = %d) has stopped running",
					prefix.c_str(),
					proc_it->second,
					proc_it->first);
		}
	}
}

// function for making the response formatting bit compact
void NodeManager::formatResponse(temoto_2::LoadResource::Response &res, int code, std::string message)
{
	// Print the message out to the console.
	if ( (code == 1) || (code == -1))
	{
		ROS_ERROR( "[node_manager/loadCb] %s", message.c_str() );
	}

	else
	{
		ROS_INFO( "[node_manager/loadCb] %s", message.c_str() );
	}

	res.code = code;
	res.message = message;
}



bool NodeManager::unloadCb(temoto_2::UnloadResource::Request &req, temoto_2::UnloadResource::Response &res)
{
	std::string prefix = node_name_ + "::" + __func__;
	ROS_INFO("%s Unload resource requested ...", prefix.c_str());

	// Lookup the requested process by its resource id.
	pid_t active_pid = 0;
	bool pid_found = false;
std::vector<temoto_id> ids_with_errors;
	for (auto& resource_id : req.resources)
	{
		for (auto& proc : running_processes_)
		{
			if (proc.second == resource_id)
			{
				active_pid = proc.first;
				pid_found = true;
				break;
			}
		}
		if (!pid_found)
		{
			ROS_ERROR("%s unable to obtain PID for resource with id %d", res.resource_id);
			return false;
		}


		// Kill the process
		ROS_INFO( "%s killing the child process: %s (PID = %d)",
				prefix.c_str(),
				formatRequest(req).c_str(),
				active_pid);

		// TODO: Check the returned value
		kill(active_pid, SIGINT);

		// Remove the process from the map
		running_processes_.erase(active_pid);
	}
}


bool NodeManager::loadCb(temoto_2::LoadResource::Request& req,
		temoto_2::LoadResource::Response& res)
{
	ROS_INFO("loadCb reached");
	// Name of the method, used for making debugging a bit simpler
	std::string prefix = node_name_ + "::" + __func__;

	// Get the service parameters
	const std::string& action = req.action;
	const std::string& package = req.package;
	const std::string& name = req.name;

	ROS_INFO("%s Received a 'LoadResource' service request: %s ...", prefix.c_str(), formatRequest(req).c_str());

	// Validate the action command. 
	if ( std::find(validActions.begin(), validActions.end(), action) == validActions.end() )
	{
		ROS_INFO("%s Action '%s' is not supported ...", prefix.c_str(), action.c_str());
		return false;
	}

	// TODO: Check if the package and node/launchfile exists
	// If everything is fine, run the commands

	// Fork the parent process
	std::cout << "forking the process ..." << std::endl;
	pid_t PID = fork();

	// Child process
	if (PID == 0)
	{
		// Execute the requested process
		execlp(action.c_str(), action.c_str(), package.c_str(), name.c_str(), (char*) NULL);
	}

	// Only parent gets here
	std::cout << "Process forked. Child PID: " << PID << std::endl;

	// Insert the pid to map of running processes
	running_processes_.insert ( {PID, req.resource_id} );

	// Fill response
	res.code = 0;
	res.message = "Command executed successfully.";
	return true;
}

} // namespace node_manager
