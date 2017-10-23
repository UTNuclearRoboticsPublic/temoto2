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
class TaskTerminal: public Task
{
public:

    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * Inherited methods that have to be implemented /START
     * * * * * * * * * * * * * * * * * * * * * * * * */

    TaskTerminal()
    {
        // Do something here if needed
        ROS_INFO("TaskTerminal constructed");
    }

    // TODO: this signature is here for compatibility purposes.
	// It is depricated, and is subjected to removal!
    bool startTask()
    {
		std::vector<boost::any> arguments;
		return startTask(0, arguments);
    }

    // starts the task
    bool startTask(int subtaskNr, std::vector<boost::any> arguments)
    {
        // Check if arguments vector contains no arguments
        if (arguments.size() > 0)
        {
			ROS_ERROR("[TaskTerminal/startTask]: Terminal works with no arguments only,"
				" but got %ld", arguments.size());
            return false;
        }

        // Name of the method, used for making debugging a bit simpler
        const std::string method_name_ = "startTask";
        std::string prefix = common::generateLogPrefix("", this->class_name_, method_name_);

        // Build a gesture specifier
        // TODO: This shoud be done via speech specifier helper class
        std::vector <temoto_2::SpeechSpecifier> speech_specifiers;
        temoto_2::SpeechSpecifier speech_specifier;
        speech_specifier.dev = "device";
        speech_specifier.type = "text";
        speech_specifiers.push_back(speech_specifier);

        // Subscribe to gesture topic
        try
        {
            hci_.getSpeech(speech_specifiers, &TaskTerminal::speech_callback, this);
        }

        catch( error::ErrorStackUtil& e )
        {
            e.forward( prefix );
            this->error_handler_.append(e);
        }

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

    // Callback for processing speech
    void speech_callback(std_msgs::String msg)
    {
        //ROS_INFO("Speech callback got: %s", msg.data.c_str());
    }

    ~TaskTerminal()
    {
        ROS_INFO("[TaskTerminal::~TaskTerminal]TaskTerminal destructed");
    }

private:

    /**
     * @brief class_name_
     */
    std::string class_name_ = "TaskTerminal";

    // Human context interface object
    HumanContextInterface <TaskTerminal> hci_;

};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(TaskTerminal, Task);
