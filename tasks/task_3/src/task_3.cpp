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
#include "context_manager/human_context_interface.h"

// First implementaton
class Imp_task_3: public Task
{
public:

    // Human context interface object
    Human_context_interface <Imp_task_3> hci_;

    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * Inherited methods that have to be implemented /START
     * * * * * * * * * * * * * * * * * * * * * * * * */

    Imp_task_3()
    {
        // Do something here if needed
        ROS_INFO("Imp_task_3 constructed");
    }

    // startTask without arguments
    int startTask()
    {
        // Subscribe to gesture topic
        if ( hci_.getGestures( &Imp_task_3::gesture_callback, this ) )
        {
            ROS_ERROR("[Imp_task_3]: getGestures failed");
        }

        return getValue();
    }

    // startTask with arguments
    int startTask(int subtaskNr, std::vector<boost::any> arguments )
    {
        // Check if arguments vector contains expected amount of arguments
        if (arguments.size() != numberOfArguments)
        {
            std::cerr << "[Imp_task_3/startTask]: Wrong number of arguments. Expected: "
                      << numberOfArguments  << " but got: " << arguments.size() << '\n';

            return 1;
        }

        // If it does, try to cast the arguments
        try
        {
            arg0 = boost::any_cast<int>(arguments[0]);
            arg1 = boost::any_cast<std::string>(arguments[1]);
            arg2 = boost::any_cast<double>(arguments[2]);

            // Print them out
            std::cout << arg0 << '\n';
            std::cout << arg1 << '\n';
            std::cout << arg2 << '\n';

            return 0;
        }
        catch (boost::bad_any_cast &e)
        {
            std::cerr << "[Imp_task_3/startTask]: " << e.what() << '\n';
            return 1;
        }
    }

    int pauseTask()
    {
        return 2;
    }

    int stopTask()
    {
        return 3;
    }

    std::string getDescription()
    {
        return description;
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
            boost::any retArg0 = arg0;
            boost::any retArg1 = arg1;
            boost::any retArg2 = arg2;

            solutionVector.push_back(retArg0);
            solutionVector.push_back(retArg1);
            solutionVector.push_back(retArg2);
        }

        return solutionVector;
    }

    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * Inherited methods that have to be implemented / END
     * * * * * * * * * * * * * * * * * * * * * * * * */

    // Callback for processing gestures
    void gesture_callback(std_msgs::String msg)
    {
        ROS_INFO("Gesture callback got: %s", msg.data.c_str());
    }

    int getValue()
    {
        return 6060;
    }

    ~Imp_task_3()
    {
        ROS_INFO("Imp_task_3 destructed");
    }

private:
    int numberOfArguments = 3;

    int arg0;
    std::string arg1;
    double arg2;

};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(Imp_task_3, Task);
