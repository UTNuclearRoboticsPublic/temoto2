/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *	Sample Task class that utilizes the Temoto 2.0 architecture.
 *
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Things that have to be included
#include "TTP/base_task/base_task.h"                 				 // The base task
#include <class_loader/class_loader.h>                       // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "algorithm_manager/algorithm_manager_interface.h"

// First implementaton
class AlgorithmManagerTests: public TTP::BaseTask
{
public:

/* * * * * * * * * * * * * * * * * * * * * * * * *
 * Inherited methods that have to be implemented /START
 * * * * * * * * * * * * * * * * * * * * * * * * */

AlgorithmManagerTests()
{
  // Do something here if needed
  TASK_INFO("AlgorithmManagerTests constructed");
}

// startTask with arguments
bool startTask(TTP::TaskInterface task_interface)
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
  }

  return true;
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
  std::string  what_0_word_out;
  std::string  what_0_data_0_out;

  // </ AUTO-GENERATED, DO NOT MODIFY >

// -------------------------------< USER CODE >-------------------------------

  // Name of the method, used for making debugging a bit simpler
  std::string prefix = common::generateLogPrefix(subsystem_name_, class_name_, __func__);

  try
  {
    // Initialize the sensor manager interface
    ami_.initialize(this);

    TASK_INFO_STREAM(prefix << " Starting the algorithm manager task");

    // Start the test algorithm
    AlgorithmTopicsReq requested_topics;
    requested_topics.addInputTopic("type_0","in_remapped_topic_0");
    requested_topics.addInputTopicType("type_1");
    requested_topics.addOutputTopic("type_0","/out_remapped_topic_0");

    AlgorithmTopicsRes responded_topics = ami_.startAlgorithm("test", requested_topics);

    // Pass the camera topic to the output
    what_0_word_out = what_0_word_in;
    what_0_data_0_out = "blah";
  }
  catch( error::ErrorStack& e )
  {
      error_handler_.forwardAndAppend(e, prefix);
  }

// -------------------------------</ USER CODE >-------------------------------

  // < AUTO-GENERATED, DO NOT MODIFY >

  TTP::Subject what_0_out("what", what_0_word_out);
  what_0_out.markComplete();
  what_0_out.data_.emplace_back("topic", boost::any_cast<std::string>(what_0_data_0_out));
  output_subjects.push_back(what_0_out);

  // </ AUTO-GENERATED, DO NOT MODIFY >
}

~AlgorithmManagerTests()
{
    TASK_INFO("AlgorithmManagerTests destructed");
}

private:

// Create context manager interface object for context manager manager
AlgorithmManagerInterface ami_;

std::string task_alias;

};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(AlgorithmManagerTests, TTP::BaseTask);
