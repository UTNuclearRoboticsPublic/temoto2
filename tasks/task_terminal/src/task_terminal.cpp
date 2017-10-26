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

// experimental testing in speech callback
#include <boost/algorithm/string.hpp>

// First implementaton
class TaskTerminal: public Task
{
public:

    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * Inherited methods that have to be implemented /START
     * * * * * * * * * * * * * * * * * * * * * * * * */

    TaskTerminal() : hci_(this)
    {
      class_name_ = "TaskTerminal";

      // Do something here if needed
      TASK_DEBUG("TaskTerminal constructed");
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
        std::string prefix = common::generateLogPrefix("", this->class_name_, __func__);
        
        try
        {
          // Initialize human context interface
          hci_.initialize();

          // Advertise the topic where core listens for commands
          core_pub_ = nh_.advertise<std_msgs::String>("/temoto_2/human_chatter", 1);

          // Build a speech specifier
          // TODO: This shoud be done via speech specifier helper class
          std::vector<temoto_2::SpeechSpecifier> speech_specifiers;
          temoto_2::SpeechSpecifier speech_specifier;
          speech_specifier.dev = "device";
          speech_specifier.type = "speech";
          speech_specifiers.push_back(speech_specifier);

          // Get speech and register callback
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
        std::string str = "healthy, maybe :)";
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
      std::string prefix = common::generateLogPrefix("", this->class_name_, __func__);
      TASK_DEBUG("%s Speech callback got: %s", prefix.c_str(), msg.data.c_str());

      std_msgs::String core_msg;

      // Some hardcoded testing
      if (msg.data == "take picture" || msg.data == "start camera" || msg.data == "show picture")
      {
        core_msg.data = "take_picture";
      }
      else if (msg.data == "stop take picture" || msg.data == "close camera" || msg.data == "close picture" || msg.data == "hide picture")
      {
        core_msg.data = "stop take_picture";
      }
      else
      {
        core_msg.data = msg.data;
      }

      if(core_pub_)
      {
        core_pub_.publish(core_msg);
      }
    }

    ~TaskTerminal()
    {
      core_pub_.shutdown();
      ROS_INFO("[TaskTerminal::~TaskTerminal]TaskTerminal destructed");
    }

private:

    /**
     * @brief class_name_
     */
    std::string class_name_;

    // Human context interface object
    HumanContextInterface <TaskTerminal> hci_;
    ros::Publisher core_pub_;
    ros::NodeHandle nh_;

};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(TaskTerminal, Task);
