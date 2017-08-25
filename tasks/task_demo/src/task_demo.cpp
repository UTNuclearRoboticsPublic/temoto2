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
#include "std_msgs/String.h"
#include "context_manager/human_context/human_context_interface.h"

// First implementaton
class TaskDemo: public Task
{
public:

    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * Inherited methods that have to be implemented /START
     * * * * * * * * * * * * * * * * * * * * * * * * */

    TaskDemo()
    {
        // Do something here if needed
        ROS_INFO("TaskDemo constructed");
    }

    // startTask without arguments
    bool startTask()
    {
        // Name of the method, used for making debugging a bit simpler
        const std::string method_name_ = "startTask";
        std::string prefix = formatMessage("", this->class_name_, method_name_);

        // Build a speech specifier
        // TODO: This shoud be done via speech specifier helper class
        /*
        std::vector <temoto_2::speechSpecifier> speechSpecifiers;
        temoto_2::speechSpecifier speechSpecifier;
        speechSpecifier.dev = "device";
        speechSpecifier.type = "text";
        speechSpecifiers.push_back(speechSpecifier);
        */

        // Build a gesture specifier
        // TODO: This shoud be done via speech specifier helper class
        std::vector <temoto_2::gestureSpecifier> gestureSpecifiers;
        temoto_2::gestureSpecifier gestureSpecifier;
        gestureSpecifier.dev = "device";
        gestureSpecifier.type = "hand";
        gestureSpecifiers.push_back(gestureSpecifier);


        // Register the speech request and bind a callback
        /*
        if ( !hci_.getSpeech(speechSpecifiers, &TaskDemo::speech_callback, this ) )
        {
            ROS_ERROR("[TaskDemo::startTask()]: getSpeech request failed");
            return 1;
        }
        */

        // Register the gesture request and bind a callback
        try
        {
            hci_.getGestures(gestureSpecifiers, &TaskDemo::gesture_callback, this );
        }

        catch( error::ErrorStackUtil& e )
        {
            e.forward( prefix );
            this->error_handler_.append(e);
        }


        ROS_INFO("Entering a endless loop that should block if this task was not threaded");
        while(!stop_task_)
        {
            std::cout << "in tha loop" << std::endl;
            ros::Duration(1).sleep();
        }

        return true;
    }

    // startTask with arguments
    bool startTask(int subtaskNr, std::vector<boost::any> arguments )
    {
        // Check if arguments vector contains expected amount of arguments
        if (arguments.size() != numberOfArguments)
        {
            std::cerr << "[TaskDemo::startTask]: Wrong number of arguments. Expected: "
                      << numberOfArguments  << " but got: " << arguments.size() << '\n';

            return false;
        }

        // If it does, try to cast the arguments
        try
        {
            arg_0 = boost::any_cast<std::string>(arguments[0]);
            print_ = false;
            startTask();
            return true;

        }
        catch (boost::bad_any_cast &e)
        {
            std::cerr << "[TaskStop::startTask]: " << e.what() << '\n';
            return false;
        }
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

    // Callback for processing speech
    void speech_callback(std_msgs::String msg)
    {
        ROS_INFO("Speech callback got: %s", msg.data.c_str());
    }

    // Callback for processing gestures
    void gesture_callback( human_msgs::Hands gesture )
    {
        if (print_)
        {
            //ROS_INFO("Gesture callback got: %f", gesture.data);
            std::cout << "Spammfest: " << gesture << std::endl;
        }
    }

    ~TaskDemo()
    {
        ROS_INFO("[TaskDemo::~TaskDemo]TaskDemo destructed");
    }

private:

    // Human context interface object
    HumanContextInterface <TaskDemo> hci_;

    /**
     * @brief class_name_
     */
    std::string class_name_ = "TaskDemo";

    int numberOfArguments = 1;
    std::string arg_0;

    bool print_ = true;

};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(TaskDemo, Task);
