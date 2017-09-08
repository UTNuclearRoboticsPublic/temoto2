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
//#include "common/tools.h"
#include "node_manager/node_manager.h"
//#include "temoto_2/nodeSpawnKill.h"
//#include "temoto_2/resourceStatus.h"

#include <stdio.h>
#include <csignal>
#include <sys/wait.h>
#include <algorithm>

namespace node_manager
{

NodeManager::NodeManager()
{
    spawn_kill_srv_ = nh_.advertiseService("spawn_kill_process", &NodeManager::spawnKillCb, this);
}

NodeManager::~NodeManager()
{}

// Function that formats a neat string from request message
std::string NodeManager::formatRequest( temoto_2::nodeSpawnKill::Request& req )
{
    return "'" + req.action + " " +req.package + " " + req.name + "'";
}

// Compare request
bool NodeManager::compareRequest( temoto_2::nodeSpawnKill::Request &req_1,
                     temoto_2::nodeSpawnKill::Request &req_2,
                     std::string action)
{
    // Compare run and launch requests
    if( action == "rosrun" || action == "roslaunch")
    {
        if( req_1.action == req_2.action &&
                req_1.package == req_2.package &&
                req_1.name == req_2.name)
        {
            return true;
        }
        return false;
    }

    // Compare kill type requests
    else if( action == "kill" )
    {
        if( req_1.package == req_2.package &&
                req_1.name == req_2.name)
        {
            return true;
        }
        return false;
	}
}


// Timer callback where running proceses are checked if they are operational
void NodeManager::update(const ros::TimerEvent&)
{

	std::string prefix = formatMessage( node_name_, "", __func__ );
	//if (pipeOpen)
	//readPipe();

	// Run through the list of running processes
	for( auto& running_process : running_processes_ )
	{
		int status;
		int kill_response = waitpid(running_process.first, &status, WNOHANG);

		std::printf( "Process '%s'(PID = %d) waitpid response = %d, status = %d\n",
				formatRequest(running_process.second).c_str(), running_process.first,
				kill_response, status );

		// If the child process has stopped running,
		if( kill_response != 0 )
		{
			ROS_ERROR( "%s Process '%s'(PID = %d) has stopped running",
					prefix.c_str(),
					formatRequest(running_process.second).c_str(),
					running_process.first );
		}
	}
}

// function for making the response formatting bit compact
void NodeManager::formatResponse(temoto_2::nodeSpawnKill::Response &res, int code, std::string message)
{
	// Print the message out to the console.
	if ( (code == 1) || (code == -1))
	{
		ROS_ERROR( "[node_manager/spawnKillCb] %s", message.c_str() );
	}

	else
	{
		ROS_INFO( "[node_manager/spawnKillCb] %s", message.c_str() );
	}

	res.code = code;
	res.message = message;
}

bool NodeManager::spawnKillCb( temoto_2::nodeSpawnKill::Request &req,
		temoto_2::nodeSpawnKill::Response &res)
{
	// Name of the method, used for making debugging a bit simpler
	std::string prefix = formatMessage( node_name_, "", __func__ );

	// Get the service parameters
	std::string action = req.action;
	std::string package = req.package;
	std::string name = req.name;

	ROS_INFO("%s Received a 'spawn_kill' service request: %s ...", prefix.c_str(), formatRequest(req).c_str());

	// Test the validity of action command. If the action string is unknown
	if ( std::find(validActions.begin(), validActions.end(), action) == validActions.end() )
	{
		formatResponse(res, 1, "Unknown action command");
		return true;
	}

	// Check if the process related to the incoming request is running or not
	bool request_active = false;
	pid_t active_pid;
	for( auto& running_process : running_processes_ )
	{
		if( compareRequest(running_process.second, req, req.action) )
		{
			request_active = true;
			active_pid = running_process.first;
			break;
		}
	}

	// TODO: Check if the package and node/launchfile exists
	// If everything is fine, run the commands

	// "Kill" command
	if (action.compare("kill") == 0)
	{
		ROS_INFO("%s Kill requested ...", prefix.c_str());

		// Check if the requested process exists in the list of running processes
		if( !request_active )
		{
			formatResponse( res, 1, "The process does not seem to be running, kill request aborted." );
			return true;
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

	else
	{
		// Check if the requested node/launchfile is already running
		if ( request_active )
		{
			formatResponse( res, 1, "The process is already running, request aborted." );
			return true;
		}

		// create a pipe
		//pipe(pipefd);

		// Fork the parent process
		std::cout << "forking the process ..." << std::endl;
		pid_t PID = fork();

		// Child process
		if (PID == 0)
		{
			// Close the read end and redirect childs standard output to parents pipe

		//	close(pipefd[0]);
		//	dup2(pipefd[1], STDOUT_FILENO);
		//	dup2(pipefd[1], STDERR_FILENO);

			// Execute the requested process
			execlp(action.c_str(), action.c_str(), package.c_str(), name.c_str(), (char*) NULL);
		}

		// Only parent gets here
		std::cout << "Process forked. Child PID: " << PID << std::endl;
		//output = fdopen(pipefd[0], "r");
		//pipeOpen = true;

		// Insert the pid to map of running processes
		running_processes_.insert ( {PID, req} );

		// Close the write end of the pipe
		//close(pipefd[1]);
		//close(pipefd[0]);
		//output = fdopen(pipefd[0], "r");
	}

	// Send response
	formatResponse( res, 0, "Command executed successfully." );
    return true;
}

} // namespace node_manager



// TODO include or discard the standard streams 
// Global filehandle for FILL IN THE BLANKS
// FILE * f;
// int pipefd[2];
// FILE* output;
// bool pipeOpen = false;

//void NodeManager::readPipe()
//{
//    char buffer[100];
//
//    if ( fgets(buffer, sizeof(buffer), output) != NULL)
//    {
//        std::cout << buffer;
//    }
//
//    //fclose(output);
//}
