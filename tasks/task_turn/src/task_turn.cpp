/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *	Sample Task class that utilizes the Temoto 2.0 architecture.
 *
 *	TASK DESCRIPTION:
 *		* Demonstrate dynamic subscription
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Things that have to be included
#include "temoto_2/base_task/task.h"                 // The base task
#include <class_loader/class_loader.h>               // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "std_msgs/Int32.h"

// First implementaton
class Turn_task: public Task
{
public:

    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * Inherited methods that have to be implemented /START
     * * * * * * * * * * * * * * * * * * * * * * * * */

    Turn_task() :
    blinker_publisher_(n_.advertise<std_msgs::Int32>("blink", 5))
    {
        // Do something here if needed
        ROS_DEBUG("Turn_task constructed");
        ros::spinOnce();

        // Wait for the publisher to come up
        ros::Duration(0.5).sleep();
    }

    // startTask without arguments
    int startTask()
    {
        return 0;
    }

    // startTask with arguments
    int startTask(int subtaskNr, std::vector<boost::any> arguments )
    {
        // Check if arguments vector contains expected amount of arguments
        if (arguments.size() != numberOfArguments)
        {
            std::cerr << "[Turn_task/startTask]: Wrong number of arguments. Expected: "
                      << numberOfArguments  << " but got: " << arguments.size() << '\n';

            return 1;
        }

        // If it does, try to cast the arguments
        try
        {
            arg0 = boost::any_cast<std::string>(arguments[0]);
            std_msgs::Int32 msg;

            if (arg0.compare("on") == 0)
            {
                std::cout << "turning the blinker ON" << std::endl;
                msg.data = 1;
                blinker_publisher_.publish(msg);
                //ros::spinOnce();
            }

            else if (arg0.compare("off") == 0)
            {
                std::cout << "turning the blinker OFF" << std::endl;
                msg.data = 9;
                blinker_publisher_.publish(msg);
                //ros::spinOnce();
            }

            return 0;
        }
        catch (boost::bad_any_cast &e)
        {
            std::cerr << "[Turn_task/startTask]: " << e.what() << '\n';
            return 1;
        }
    }

    int pauseTask()
    {
        return 0;
    }

    int stopTask()
    {
        return 0;
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
            boost::any retArg0 = result;
            solutionVector.push_back(retArg0);
        }

        return solutionVector;
    }

    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * Inherited methods that have to be implemented / END
     * * * * * * * * * * * * * * * * * * * * * * * * */

    ~Turn_task()
    {
        ros::Duration(0.5).sleep();
        ROS_DEBUG("[Turn_task] Turn_task destructed");
    }

private:
    ros::NodeHandle n_;
    ros::Publisher blinker_publisher_;

    int numberOfArguments = 1;

    std::string arg0;
    int result;
};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(Turn_task, Task);
