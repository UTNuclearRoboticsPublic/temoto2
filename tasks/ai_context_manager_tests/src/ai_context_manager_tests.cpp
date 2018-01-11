/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *	Sample Task class that utilizes the Temoto 2.0 architecture.
 *
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Things that have to be included
#include "TTP/base_task/base_task.h"                 				 // The base task
#include <class_loader/class_loader.h>                                   // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "context_manager/context_manager_interface.h"
#include <visualization_msgs/Marker.h>

// First implementaton
class ContextManagerTests: public TTP::BaseTask
{
public:

/* * * * * * * * * * * * * * * * * * * * * * * * *
 * Inherited methods that have to be implemented /START
 * * * * * * * * * * * * * * * * * * * * * * * * */

ContextManagerTests()
{
  // Do something here if needed
  TASK_INFO("ContextManagerTests constructed");
}

// startTask with arguments
void startTask(TTP::TaskInterface task_interface)
{
  // TODO: This is a hack for Veiko
  task_alias = task_interface.alias_;

  // < AUTO-GENERATED, DO NOT MODIFY >
  input_subjects = task_interface.input_subjects_;
  switch(task_interface.id_)
  {
      // Interface 0
      case 0:
          startInterface_0();
      break;

      // Interface 1
      case 1:
          startInterface_1();
      break;
  }
  // </ AUTO-GENERATED, DO NOT MODIFY >
}

std::string getStatus()
{
  std::string str = "healthy";
  return str;
}

std::vector<TTP::Subject> getSolution()
{
  return output_subjects;
}


/* * * * * * * * * * * * * * * * * * * * * * * * *
 * Inherited methods that have to be implemented / END
 * * * * * * * * * * * * * * * * * * * * * * * * */

/*
 * Interface 0 body
 */
void startInterface_0()
{
  // < AUTO-GENERATED, DO NOT MODIFY >

  // Extracting input subjects
  TTP::Subject what_0_in = TTP::getSubjectByType("what", input_subjects);
  std::string  what_0_word_in = what_0_in.words_[0];

  // Creating output variables
  std::string what_0_word_out;
  std::string what_0_data_0_out;

  // </ AUTO-GENERATED, DO NOT MODIFY >

// -------------------------------< USER CODE >-------------------------------

  // Initialize the sensor manager interface
  cmi_.initialize(this);

  // Start tracking the object and return the topic to the object
  what_0_data_0_out = cmi_.trackObject(what_0_word_in);
  what_0_word_out = "object container";

// -------------------------------</ USER CODE >-------------------------------

  // < AUTO-GENERATED, DO NOT MODIFY >

  TTP::Subject what_0_out("what", what_0_word_out);
  what_0_out.markComplete();
  what_0_out.data_.emplace_back("topic", boost::any_cast<std::string>(what_0_data_0_out));
  output_subjects.push_back(what_0_out);

  // </ AUTO-GENERATED, DO NOT MODIFY >
}

/*
 * Interface 1 body
 */
void startInterface_1()
{
  // < AUTO-GENERATED, DO NOT MODIFY >

  // Extracting input subjects
  TTP::Subject what_0_in = TTP::getSubjectByType("what", input_subjects);
  std::string  what_0_word_in = what_0_in.words_[0];

  // Creating output variables
  std::string  what_0_word_out;

  // </ AUTO-GENERATED, DO NOT MODIFY >

// -------------------------------< USER CODE >-------------------------------

  // Name of the method, used for making debugging a bit simpler
  std::string prefix = common::generateLogPrefix("", this->class_name_, __func__);

  try
  {
    // Initialize the context manager interface
    cmi_.initialize(this);

    // Start a tracker
    TopicContainer tracker_topics = cmi_.startTracker("artags");

    // Pass the name of the object to the output
    what_0_word_out = tracker_topics.getOutputTopic("marker_data");

    std::cout << "GOT TOPIC: " << what_0_word_out << std::endl;
  }
  catch( error::ErrorStack& error_stack )
  {
    SEND_ERROR(error_stack);
  }

// -------------------------------</ USER CODE >-------------------------------

  // < AUTO-GENERATED, DO NOT MODIFY >

  TTP::Subject what_0_out("what", what_0_word_out);
  what_0_out.markComplete();
  output_subjects.push_back(what_0_out);

  // </ AUTO-GENERATED, DO NOT MODIFY >
}

~ContextManagerTests()
{
    TASK_INFO("ContextManagerTests destructed");
}

private:

// Nodehandle
ros::NodeHandle n_;

// Create context manager interface object for context manager manager
context_manager::ContextManagerInterface <ContextManagerTests> cmi_;

std::string task_alias;

};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(ContextManagerTests, TTP::BaseTask);
