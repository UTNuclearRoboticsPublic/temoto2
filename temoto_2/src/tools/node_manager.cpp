/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *          MASSIVE TODO: * CATCH ALL EXCEPTIONS !!!
 *                        * implement interprocess piping service
 *                          that starts streaming the std::out of
 *                          a requested process.
 *                        * organize your sh*t
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "temoto_core/common/request_container.h"
#include "std_msgs/String.h"
#include "temoto_2/nodeSpawnKill.h"
#include "temoto_2/resourceStatus.h"

#include <stdio.h>
#include <csignal>
#include <sys/wait.h>
#include <algorithm>

// Global filehandle for FILL IN THE BLANKS
FILE * f;
std::map <std::string, pid_t> running_processes;

pid_t pid;
int pipefd[2];
FILE* output;
char line[256];
int status;
bool pipeOpen = false;

const std::vector<std::string> validActions = {"rosrun", "roslaunch", "kill"};

// Timer callback where running proceses are checked if they are operational
void timerCallback( const ros::TimerEvent& )
{
    for( auto& running_process : running_processes )
    {
        int status;
        int kill_response = waitpid(running_process.second, &status, WNOHANG);

        std::printf( "Process '%s'(PID = %d) waitpid response = %d, status = %d\n",running_process.first.c_str(), running_process.second, kill_response, status );
        if( kill_response != 0 )
        {
            ROS_ERROR("Process '%s'(PID = %d) has stopped running",running_process.first.c_str(), running_process.second );
        }
    }
}

// function for making the response formatting bit compact
void formatResponse(temoto_2::nodeSpawnKill::Response &res, int code, std::string message)
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

bool spawnKillCb(temoto_2::nodeSpawnKill::Request &req,
                   temoto_2::nodeSpawnKill::Response &res)
{
    // Get the service parameters
    std::string action = req.action;
    std::string package = req.package;
    std::string name = req.name;

    ROS_INFO("[node_manager/spawnKillCb] Received a 'spawn_kill' service request: '%s %s %s' ...", action.c_str(), package.c_str(), name.c_str());

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
        ROS_INFO("[node_manager/spawnKillCb] Kill requested ...");

        // Check if the requested process exists in the list of running processes
        if ( running_processes.find(name) == running_processes.end() )
        {
            formatResponse( res, 1, "The process does not seem to be running, kill request aborted." );
            return true;
        }

        // Kill the process
        std::cout << "killing the child process '" << name << "' (PID: " << running_processes[name] << ")" << std::endl;

        // TODO: Check the returned value
        kill(running_processes[name], SIGINT);

        // Remove the process from the map
        running_processes.erase(name);
    }

    else
    {
        // Check if the requested node/launchfile is already running
        if ( running_processes.find(name) != running_processes.end() )
        {
            formatResponse( res, 1, "The process is already running, request aborted." );
            return true;
        }

        ROS_INFO("[node_manager/callback_1] '%s %s %s' requested ...", action.c_str(), package.c_str(), name.c_str());

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
        //output = fdopen(pipefd[0], "r");
        //pipeOpen = true;

        // Insert the pid to map of running processes
        running_processes.insert ( {name, PID} );

        // Close the write end of the pipe
        close(pipefd[1]);
        close(pipefd[0]);
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_manager");
    ros::NodeHandle n;

    ros::ServiceServer spawn_kill_srv = n.advertiseService("spawn_kill_process", spawnKillCb);
    ros::Timer timer = n.createTimer(ros::Duration(1), timerCallback);

    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        /*
        if (pipeOpen)
            readPipe();
        */
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}



