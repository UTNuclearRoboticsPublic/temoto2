/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *	Sample Task class that utilizes the Temoto 2.0 architecture.
 *
 *	TASK DESCRIPTION:
 *		* Demonstrate dynamic subscription
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Things that have to be included
#include "base_task/task.h"                 // The base task
#include <class_loader/class_loader.h>               // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "std_msgs/String.h"

// First implementaton
class Add_task: public Task
{
public:

    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * Inherited methods that have to be implemented /START
     * * * * * * * * * * * * * * * * * * * * * * * * */

    Add_task()
    {
        // Do something here if needed
        ROS_DEBUG("Add_task constructed");
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
            std::cerr << "[Add_task/startTask]: Wrong number of arguments. Expected: "
                      << numberOfArguments  << " but got: " << arguments.size() << '\n';

            return false;
        }

        // If it does, try to cast the arguments
        try
        {
            arg0 = boost::any_cast<int>(arguments[0]);
            arg1 = boost::any_cast<int>(arguments[1]);

            // THE HOLY TASK ITSELF
            result = arg0 + arg1;
            std::cout << arg0 << " + " << arg1 << " = " << result << std::endl;

            return true;
        }
        catch (boost::bad_any_cast &e)
        {
            std::cerr << "[Add_task/startTask]: " << e.what() << '\n';
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

        // Check the subtask number
        if ( subtaskNr == 0)
        {
            boost::any retArg0 = result;
            solutionVector.push_back(retArg0);
        }

        return solutionVector;
    }

    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * Inherited methods that have to be implemented / END
     * * * * * * * * * * * * * * * * * * * * * * * * */

    ~Add_task()
    {
        ROS_DEBUG("[Add_task] Add_task destructed");
    }

private:
    int numberOfArguments = 2;

    int arg0;
    int arg1;
    int result;
};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(Add_task, Task);
