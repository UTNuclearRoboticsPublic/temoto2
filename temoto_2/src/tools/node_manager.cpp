/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
*
* The purpose of this code is to test the ros::master and xmlrpc api
* more percisely, the purpose is to close any given node 
*	programmatically. When the node is started, the main loop waits
* for an input string that is then passed to callback_1. Callback_1
* supports 2 commands and asks for additional input (name of the
* node, port number, etc)
*
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *          MASSIVE TODO: CATCH ALL EXEPTIONS !!!
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <stdio.h>
#include <csignal>
#include <algorithm>

#include "std_msgs/String.h"
#include <ros/xmlrpc_manager.h>
#include "temoto_2/node_spawn_kill.h"

#include "core/common.h"

FILE * f;

std::map <std::string, pid_t> runningProcesses;

pid_t pid;
int pipefd[2];
FILE* output;
char line[256];
int status;
bool pipeOpen = false;

const std::vector<std::string> validActions = {"rosrun", "roslaunch", "kill"};

// function for making the response formatting bit compact
void formatResponse(temoto_2::node_spawn_kill::Response &res, int code, std::string message)
{
    // Print the message out to the console.
    if ( (code == 1) || (code == -1))
    {
        ROS_ERROR( "[spawn_kill_cb] %s", message.c_str() );
    }

    else
    {
        ROS_INFO( "[spawn_kill_cb] %s", message.c_str() );
    }

    res.code = code;
    res.message = message;
}

bool spawn_kill_cb(temoto_2::node_spawn_kill::Request &req,
                   temoto_2::node_spawn_kill::Response &res)
{
    // Get the service parameters
    std::string action = req.action;
    std::string package = req.package;
    std::string name = req.name;

    ROS_INFO("[spawn_kill_cb] Received a spawn_kill service request '%s %s %s' ...", action.c_str(), package.c_str(), name.c_str());

    // Test the validity of action command. If the action string is unknown
    if ( std::find(validActions.begin(), validActions.end(), action) == validActions.end() )
    {
        formatResponse(res, 1, "Unknown action command.");
        return true;
    }

    // TODO: Check if the package and node/launchfile exists
    // If everything is fine, run the commands

    // "Kill" command
    if (action.compare("kill") == 0)
    {
        ROS_INFO("[callback_1] Kill requested ...");

        // Check if the requested process exists in the list of running processes
        if ( runningProcesses.find(name) == runningProcesses.end() )
        {
            formatResponse( res, 1, "The process does not seem to be running, kill request aborted." );
            return true;
        }

        // Kill the process
        std::cout << "killing the child process '" << name << "' (PID: " << runningProcesses[name] << ")" << std::endl;

        // TODO: Check the returned value
        kill(runningProcesses[name], SIGINT);

        // Remove the process from the map
        runningProcesses.erase(name);
    }

    else
    {
        // Check if the requested node/launchfile is already running
        if ( runningProcesses.find(name) != runningProcesses.end() )
        {
            formatResponse( res, 1, "The process is already running, request aborted." );
            return true;
        }

        ROS_INFO("[callback_1] '%s %s %s' requested ...", action.c_str(), package.c_str(), name.c_str());

        // create a pipe
        pipe(pipefd);

        // Fork the parent process
        std::cout << "forking the process ..." << std::endl;
        pid_t PID = fork();

        // Child process
        if (PID == 0)
        {
            // Close the read end and redirect childs standard output to parents pipe
            close(pipefd[0]);
            dup2(pipefd[1], STDOUT_FILENO);
            dup2(pipefd[1], STDERR_FILENO);

            // Execute the requested process
            execlp(action.c_str(), action.c_str(), package.c_str(), name.c_str(), (char*) NULL);
        }

        // Only parent gets here
        std::cout << "Process forked. Child PID: " << PID << std::endl;
        output = fdopen(pipefd[0], "r");
        pipeOpen = true;

        // Insert the pid to map of running processes
        runningProcesses.insert ( {name, PID} );

        // Close the write end of the pipe
        close(pipefd[1]);
        //output = fdopen(pipefd[0], "r");
    }

    // Send response
    formatResponse( res, 0, "Command executed successfully." );
    return true;
}

void readPipe()
{
    char buffer[100];

    if ( fgets(buffer, sizeof(buffer), output) != NULL)
    {
        std::cout << buffer;
    }

    //fclose(output);
}

void callback_1 (const std_msgs::String& command)
{
    XmlRpc::XmlRpcValue request;
    XmlRpc::XmlRpcValue response;
    XmlRpc::XmlRpcValue payload;

    if (command.data.compare("lookupNode") == 0)
    {
        ROS_INFO("[callback_1] %s requested. Specify the name of the node: ", command.data.c_str());

        std::string node_name;
        std::getline(std::cin, node_name);

        request[0] = ros::this_node::getName();
        request[1] = node_name;

        ROS_INFO("[callback_1] sending '%s' for %s ...", command.data.c_str(), node_name.c_str());

        bool success = ros::master::execute(command.data.c_str(), request,
                response, payload, true);

        ROS_INFO("[callback_1] sending was %s", success?"sucessful":"unsuccessful");

        std::cout << "request: " << request << std::endl;
        std::cout << "response: " << response << std::endl;
        std::cout << "payload: " << payload << std::endl;
    }

    // Shut the node down. there is no master call for shutdown, hence it has to
    // be called directly via xmlrpc that is embedded inside master api anyways

    else if (command.data.compare("shutdown") == 0)
    {
        ROS_INFO("[callback_1] %s requested. Specify the name of the node: ", command.data.c_str());

        std::string host;
        uint32_t port;
        std::string uri;
        request[0] = ros::this_node::getName();
        request[1] = "because I can";

        std::cout << "Enter port nr: ";
        std::cin >> port;
        std::cout << std::endl;

        XmlRpc::XmlRpcClient *c = ros::XMLRPCManager::instance()->getXMLRPCClient("localhost", port, "/");
        bool success = c->execute(command.data.c_str(), request, response);

        ROS_INFO("[callback_1] sending was %s", success?"sucessful":"unsuccessful");
        std::cout << "request: " << request << std::endl;
        std::cout << "response: " << response << std::endl;
    }

    // New proc via fork + exec
    else if (command.data.compare("t2") == 0)
    {
        ROS_INFO("[callback_1] %s requested ...", command.data.c_str());

        // create a pipe
        pipe(pipefd);

        // Fork the parent process
        std::cout << "forking the process ..." << std::endl;
        pid = fork();

        // Child process
        if (pid == 0)
        {
            // Let's redirect its standard output to our pipe and replace process with tail
            close(pipefd[0]);
            dup2(pipefd[1], STDOUT_FILENO);
            dup2(pipefd[1], STDERR_FILENO);
            //execlp("rosrun", "rosrun", "rospy_tutorials", "talker.py", (char*) NULL);
            execlp("roslaunch", "roslaunch", "temoto_2", "test_1.launch", (char*) NULL);
        }

        //Only parent gets here. Listen to what the tail says
        std::cout << "PID of the child is: " << pid << std::endl;

        close(pipefd[1]);
        output = fdopen(pipefd[0], "r");
/*
        while(fgets(line, sizeof(line), output)) //listen to what tail writes to its standard output
        {
        //if you need to kill the tail application, just kill it:
          if(something_goes_wrong)
            kill(pid, SIGKILL);
        }
*/
        //or wait for the child process to terminate
        //waitpid(pid, &status, 0);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_manager");
    ros::NodeHandle n;

    ros::ServiceServer spawn_kill_srv = n.advertiseService("spawn_kill_process", spawn_kill_cb);

    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        if (pipeOpen)
            readPipe();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}



