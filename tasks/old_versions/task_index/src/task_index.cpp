/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *	Task that calls the "index_tasks" service of the Task Handler
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Things that have to be included
#include "base_task/task.h"                 				 // The base task
#include <class_loader/class_loader.h>               // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "temoto_2/indexTasks.h"


// First implementaton
class TaskIndex: public Task
{
public:

    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * Inherited methods that have to be implemented /START
     * * * * * * * * * * * * * * * * * * * * * * * * */

    TaskIndex()
    {
        index_tasks_client_ = n_.serviceClient<temoto_2::indexTasks>("core/index_tasks");
        ROS_INFO("TaskIndex constructed");
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
        if (arguments.size() != number_of_arguments)
        {
            std::cerr << "[TaskIndex::startTask]: Wrong number of arguments. Expected: "
                      << number_of_arguments  << " but got: " << arguments.size() << '\n';

            return false;
        }

        // If it does, try to cast the arguments
        try
        {
            arg_0 = boost::any_cast<std::string>(arguments[0]);

            ROS_INFO("[TaskIndex::startTask] Trying to index tasks");

            // Create a service message
            temoto_2::indexTasks index_tasks_srv;
            index_tasks_srv.request.directory = "/home/robert/catkin_ws/src/temoto2/tasks/";

            // Call the server
            index_tasks_client_.call(index_tasks_srv);

            ROS_INFO("[TaskIndex::startTask] '/core/index_tasks' service responded: %s", index_tasks_srv.response.message.c_str());

            // Check the result
            if (index_tasks_srv.response.code == 0)
                return true;
            else
                return false;

        }
        catch (boost::bad_any_cast &e)
        {
            std::cerr << "[TaskIndex::startTask]: " << e.what() << '\n';
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

    ~TaskIndex()
    {
        ROS_INFO("[TaskIndex::~TaskIndex] TaskIndex destructed");
    }

private:

    ros::NodeHandle n_;
    ros::ServiceClient index_tasks_client_;

    int number_of_arguments = 1;
    std::string arg_0;
};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(TaskIndex, Task);
