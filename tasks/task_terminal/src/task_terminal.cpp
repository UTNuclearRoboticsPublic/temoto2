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
#include "TTP/base_task/base_task.h"                 				 // The base task
#include <class_loader/class_loader.h>                                   // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "context_manager/human_context/human_context_interface.h"

// experimental testing in speech callback
#include <boost/algorithm/string.hpp>

// First implementaton
class TaskTerminal: public TTP::BaseTask
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

// startTask
bool startTask(TTP::TaskInterface task_interface)
{
    // * AUTO-GENERATED, DO NOT MODIFY *
    input_subjects = task_interface.input_subjects_;

    switch(task_interface.id_)
    {
    // Interface 0
    case 0:
        startInterface_0();
    break;
    }

    return true;
    // * AUTO-GENERATED, DO NOT MODIFY *
}

/*
 * Interface 0 body
 */
void startInterface_0()
{
    // < AUTO-GENERATED, DO NOT MODIFY >

    // Extracting input subjects
    TTP::Subject what_0_in = TTP::getSubjectByType("what", input_subjects);
    std::string  what_0_word_in = what_0_in.words_[0];

    // </ AUTO-GENERATED, DO NOT MODIFY >

// --------------------------------< USER CODE >-------------------------------

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

// --------------------------------</ USER CODE >-------------------------------
}

std::string getStatus()
{
    std::string str = "healthy, maybe :)";
    return str;
}

std::vector<TTP::Subject> getSolution()
{
    return output_subjects;
}

/* * * * * * * * * * * * * * * * * * * * * * * * *
 * Inherited methods that have to be implemented / END
 * * * * * * * * * * * * * * * * * * * * * * * * */

// Callback for processing speech
void speech_callback(std_msgs::String msg)
{
    std::string prefix = common::generateLogPrefix("", this->class_name_, __func__);
    TASK_DEBUG("%s Speech callback got: %s", prefix.c_str(), msg.data.c_str());

    // Publish if the publisher is active
    if(core_pub_)
    {
        core_pub_.publish(msg);
    }
}

~TaskTerminal()
{
  core_pub_.shutdown();
  ROS_INFO("[TaskTerminal::~TaskTerminal] TaskTerminal destructed");
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
CLASS_LOADER_REGISTER_CLASS(TaskTerminal, TTP::BaseTask);
