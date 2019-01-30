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
#include "temoto_nlp/base_task/base_task.h"    // The base task
#include <class_loader/class_loader.h>  // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "temoto_context_manager/context_manager_interface.h"

// experimental testing in speech callback
#include <boost/algorithm/string.hpp>

// First implementaton
class TaTaskTerminal : public temoto_nlp::BaseTask
{
public:
  /* * * * * * * * * * * * * * * * * * * * * * * * *
   * Inherited methods that have to be implemented /START
   * * * * * * * * * * * * * * * * * * * * * * * * */

  TaTaskTerminal()
  {
    class_name_ = "TaTaskTerminal";

    // Do something here if needed
    TASK_DEBUG("TaTaskTerminal constructed");
  }

  // startTask
  void startTask(temoto_nlp::TaskInterface task_interface)
  {
    // * AUTO-GENERATED, DO NOT MODIFY *
    input_subjects = task_interface.input_subjects_;

    switch (task_interface.id_)
    {
      // Interface 0
      case 0:
        startInterface_0();
        break;
    }
    // * AUTO-GENERATED, DO NOT MODIFY *
  }

  /*
   * Interface 0 body
   */
  void startInterface_0()
  {
    // < AUTO-GENERATED, DO NOT MODIFY >

    // Extracting input subjects
    temoto_nlp::Subject what_0_in = temoto_nlp::getSubjectByType("what", input_subjects);
    std::string what_0_word_in = what_0_in.words_[0];

    // </ AUTO-GENERATED, DO NOT MODIFY >

    // --------------------------------< USER CODE >-------------------------------

    // Initialize context manager interface
    cmi_.initialize(this);

    // Advertise the topic where core listens for commands
    core_pub_ = nh_.advertise<std_msgs::String>("human_chatter", 1);

    // Build a speech specifier
    // TODO: This shoud be done via speech specifier helper class
    std::vector<temoto_context_manager::SpeechSpecifier> speech_specifiers;
    temoto_context_manager::SpeechSpecifier speech_specifier;
    speech_specifier.dev = "device";
    speech_specifier.type = "speech";
    speech_specifiers.push_back(speech_specifier);

    // Get speech and register callback
    cmi_.getSpeech(speech_specifiers, &TaTaskTerminal::speech_callback, this);

    // --------------------------------</ USER CODE >-------------------------------
  }

  std::string getStatus()
  {
    std::string str = "healthy, maybe :)";
    return str;
  }

  std::vector<temoto_nlp::Subject> getSolution()
  {
    return output_subjects;
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * *
   * Inherited methods that have to be implemented / END
   * * * * * * * * * * * * * * * * * * * * * * * * */

  // Callback for processing speech
  void speech_callback(std_msgs::String msg)
  {
    std::string prefix = temoto_core::common::generateLogPrefix("", this->class_name_, __func__);
    TASK_DEBUG("%s Speech callback got: %s", prefix.c_str(), msg.data.c_str());

    // Publish if the publisher is active
    if (core_pub_)
    {
      core_pub_.publish(msg);
    }
  }

  ~TaTaskTerminal()
  {
    core_pub_.shutdown();
    ROS_INFO("[TaTaskTerminal::~TaTaskTerminal] TaTaskTerminal destructed");
  }

private:
  /**
   * @brief class_name_
   */
  std::string class_name_;

  // Human context interface object
  temoto_context_manager::ContextManagerInterface<TaTaskTerminal> cmi_;
  ros::Publisher core_pub_;
  ros::NodeHandle nh_;
};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(TaTaskTerminal, temoto_nlp::BaseTask);
