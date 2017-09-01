/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *	Sample Task class that utilizes the Temoto 2.0 architecture.
 *
 *	TASK DESCRIPTION:
 *		* Demonstrate dynamic subscription
 *              * Brings up a terminal that allows to send commands to the core
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Things that have to be included
#include "base_task/task.h"                 				 // The base task
#include <class_loader/class_loader.h>                                   // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "common/temoto_id.h"
#include "std_msgs/String.h"
#include "temoto_2/stopTask.h"


// First implementaton
class TaskStop: public Task
{
public:

    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * Inherited methods that have to be implemented /START
     * * * * * * * * * * * * * * * * * * * * * * * * */

    TaskStop()
    {
        stop_task_client_ = n_.serviceClient<temoto_2::stopTask>("core/stop_task");
        ROS_INFO("TaskStop constructed");
    }

    // startTask without arguments
    bool startTask()
    {
        return true;
    }

    // startTask with arguments
    bool startTask(int subtaskNr, std::vector<boost::any> arguments )
    {
        // Check if arguments vector contains expected amount of arguments
        if (arguments.size() != numberOfArguments)
        {
            std::cerr << "[TaskStop::startTask]: Wrong number of arguments. Expected: "
                      << numberOfArguments  << " but got: " << arguments.size() << '\n';

            return false;
        }

        // If it does, try to cast the arguments
        try
        {
            arg_0 = boost::any_cast<std::string>(arguments[0]);

            ROS_INFO("[TaskStop::startTask] Trying to stop task: %s", arg_0.c_str());

            // Create a service message
            temoto_2::stopTask stop_task_srv;
            stop_task_srv.request.name = arg_0;
            stop_task_srv.request.task_id = TemotoID::UNASSIGNED_ID;

            // Call the server
            stop_task_client_.call(stop_task_srv);

            ROS_INFO("[TaskStop::startTask] '/core/stop_task' service respinded: %s", stop_task_srv.response.message.c_str());

            // Check the result
            if (stop_task_srv.response.code == 0)
                return true;
            else
                return false;

        }
        catch (boost::bad_any_cast &e)
        {
            std::cerr << "[TaskStop::startTask]: " << e.what() << '\n';
            return false;
        }
    }

    bool stopTask()
    {
        return true;
    }

    std::string getStatus()
    {
        std::string str = "healthy";
        return str;
    }

    std::vector<boost::any> getSolution( int subtaskNr )
    {
        // Construct an empty vector
        std::vector<boost::any> solutionVector;

        return solutionVector;
    }

    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * Inherited methods that have to be implemented / END
     * * * * * * * * * * * * * * * * * * * * * * * * */

    ~TaskStop()
    {
        ROS_INFO("[TaskStop::~TaskStop] TaskStop destructed");
    }

private:

    ros::NodeHandle n_;
    ros::ServiceClient stop_task_client_;

    int numberOfArguments = 1;
    std::string arg_0;
};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(TaskStop, Task);
